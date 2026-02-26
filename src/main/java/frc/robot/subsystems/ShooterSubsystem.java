package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SHOOTER SUBSYSTEM - 2026 REBUILT
 *
 * 3x Kraken X60 - DutyCycleOut ile full power.
 * Voltaj/PID yok, basit %100 guc.
 *
 *   Motor 1 (CAN 9):  +1 yon (CounterClockwise_Positive)
 *   Motor 2 (CAN 10): -1 yon (Clockwise_Positive)
 *   Motor 3 (CAN 11): -1 yon (Clockwise_Positive)
 *   Coast mode
 */
public class ShooterSubsystem extends SubsystemBase {

    public static final int MOTOR_1_CAN_ID = 9;
    public static final int MOTOR_2_CAN_ID = 10;
    public static final int MOTOR_3_CAN_ID = 11;
    public static final String CAN_BUS     = "rio";

    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFX motor3;

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private boolean running = false;
    private double output = 0.0;

    public ShooterSubsystem() {
        motor1 = new TalonFX(MOTOR_1_CAN_ID, CAN_BUS);
        motor2 = new TalonFX(MOTOR_2_CAN_ID, CAN_BUS);
        motor3 = new TalonFX(MOTOR_3_CAN_ID, CAN_BUS);

        // Motor 1: +1 yon
        TalonFXConfiguration cfg1 = new TalonFXConfiguration();
        cfg1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg1.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg1.CurrentLimits.StatorCurrentLimit = 120;
        cfg1.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg1.CurrentLimits.SupplyCurrentLimit = 70;
        motor1.getConfigurator().apply(cfg1);

        // Motor 2 & 3: -1 yon
        TalonFXConfiguration cfg2 = new TalonFXConfiguration();
        cfg2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        cfg2.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg2.CurrentLimits.StatorCurrentLimit = 120;
        cfg2.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg2.CurrentLimits.SupplyCurrentLimit = 70;
        motor2.getConfigurator().apply(cfg2);
        motor3.getConfigurator().apply(cfg2);

        stop();
        System.out.println("[Shooter] Init - CAN 9(+1), 10(-1), 11(-1) | DutyCycle FULL POWER");
    }

    /** Shooter'i calistir. speed = -1.0 ile 1.0 arasi. 1.0 = full power. */
    public void run(double speed) {
        output = Math.max(-1.0, Math.min(1.0, speed));
        running = output != 0;
        motor1.setControl(dutyCycle.withOutput(output));
        motor2.setControl(dutyCycle.withOutput(output));
        motor3.setControl(dutyCycle.withOutput(output));
    }

    /** Full power calistir (+1.0). */
    public void runFull() {
        run(1.0);
    }

    /** Durdur. */
    public void stop() {
        output = 0.0;
        running = false;
        motor1.setControl(dutyCycle.withOutput(0));
        motor2.setControl(dutyCycle.withOutput(0));
        motor3.setControl(dutyCycle.withOutput(0));
    }

    public boolean isRunning() { return running; }
    public double getOutput() { return output; }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/Running", running);
        SmartDashboard.putNumber("Shooter/Output", output);
    }
}
