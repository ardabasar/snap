package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * FEEDER SUBSYSTEM - 2026 REBUILT
 *
 * Shooter ile ayni mantik - DutyCycleOut full power.
 * Shooter ne yapiyorsa feeder da ayni seyi yapar.
 *
 * CAN 15 (rio bus), Coast mode
 */
public class FeederSubsystem extends SubsystemBase {

    public static final int MOTOR_CAN_ID = 15;
    public static final String CAN_BUS   = "rio";

    private final TalonFX motor;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private boolean running = false;
    private double output = 0.0;

    public FeederSubsystem() {
        motor = new TalonFX(MOTOR_CAN_ID, CAN_BUS);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 120;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 50;
        motor.getConfigurator().apply(cfg);

        stop();
        System.out.println("[Feeder] Init - CAN " + MOTOR_CAN_ID + " | DutyCycle FULL POWER");
    }

    /** Feeder'i calistir. speed = -1.0 ile 1.0 arasi. */
    public void run(double speed) {
        output = Math.max(-1.0, Math.min(1.0, speed));
        running = output != 0;
        motor.setControl(dutyCycle.withOutput(output));
    }

    /** Full power calistir (+1.0). Shooter ile ayni. */
    public void feed() {
        run(1.0);
    }

    /** Durdur. */
    public void stop() {
        output = 0.0;
        running = false;
        motor.setControl(dutyCycle.withOutput(0));
    }

    public boolean isRunning() { return running; }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Feeder/Running", running);
        SmartDashboard.putNumber("Feeder/Output", output);
    }
}
