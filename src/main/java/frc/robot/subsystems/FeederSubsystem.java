package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * FEEDER SUBSYSTEM - 2026 REBUILT (WCP birebir)
 * ============================================================================
 * WCP: Feeder 5000 RPM sabit hizda calisir.
 * VelocityVoltage PID ile RPM kontrolu.
 * 
 * Donanim: CAN 15 (rio bus), Coast mode
 * PID: kP=1, kI=0, kD=0, kV=12/freeSpeed
 * ============================================================================
 */
public class FeederSubsystem extends SubsystemBase {

    public static final int MOTOR_CAN_ID = 15;
    public static final String CAN_BUS   = "rio";

    /** WCP sabit feeder hizi */
    public static final double FEED_RPM = 5000.0;

    // Kraken X60 free speed: ~6000 RPM = 100 RPS
    private static final double KRAKEN_FREE_SPEED_RPS = 100.0;

    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private boolean running = false;
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    public FeederSubsystem() {
        motor = new TalonFX(MOTOR_CAN_ID, CAN_BUS);

        // WCP Feeder config birebir
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(50))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(1)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / KRAKEN_FREE_SPEED_RPS)
            );

        motor.getConfigurator().apply(config);
        stop();
        System.out.println("[Feeder] Initialized - CAN " + MOTOR_CAN_ID + " | WCP 5000 RPM");
    }

    /** Feeder'i WCP sabit hizda calistirir (5000 RPM). */
    public void feed() {
        running = true;
        motor.setControl(velocityRequest.withVelocity(RPM.of(FEED_RPM)));
    }

    /** Feeder'i belirtilen RPM'de calistirir. */
    public void setTargetRPM(double rpm) {
        running = true;
        motor.setControl(velocityRequest.withVelocity(RPM.of(Math.abs(rpm))));
    }

    /** Feeder'i percent output ile calistirir (WCP setPercentOutput). */
    public void setPercentOutput(double percent) {
        running = percent != 0;
        motor.setControl(voltageRequest.withOutput(Volts.of(percent * 12.0)));
    }

    /** Feeder'i durdurur. */
    public void stop() {
        running = false;
        setPercentOutput(0.0);
    }

    public double getRPM() {
        return Math.abs(motor.getVelocity().getValue().in(RPM));
    }
    public boolean isRunning() { return running; }

    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("Feeder/RPM", Math.round(getRPM()));
        SmartDashboard.putNumber("Feeder/Current",
            Math.round(motor.getStatorCurrent().getValue().in(Amps) * 10.0) / 10.0);
        SmartDashboard.putBoolean("Feeder/Running", running);
    }
}
