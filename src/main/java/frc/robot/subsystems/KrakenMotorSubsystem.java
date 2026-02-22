package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenMotorSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final String name;

    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Temperature> tempSignal;

    private double targetSpeed = 0.0;
    private double targetVoltage = 0.0;

    private static final int DASHBOARD_INTERVAL = 16;
    private int loopCount = 0;

    public KrakenMotorSubsystem(int canId) {
        this("Kraken", canId, "rio");
    }

    public KrakenMotorSubsystem(int canId, String canBus) {
        this("Kraken", canId, canBus);
    }

    public KrakenMotorSubsystem(String name, int canId, String canBus) {
        this.name = name;
        motor = new TalonFX(canId, new CANBus(canBus));
        motor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());

        velocitySignal = motor.getRotorVelocity();
        voltageSignal  = motor.getMotorVoltage();
        currentSignal  = motor.getStatorCurrent();
        tempSignal     = motor.getDeviceTemp();

        stop();
        System.out.println("[" + name + "] Motor initialized — CAN ID: " + canId + ", Bus: " + canBus);
    }

    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        targetVoltage = 0.0;
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));

        SmartDashboard.putNumber(name + "/TargetSpeed", targetSpeed);
        SmartDashboard.putBoolean(name + "/Active", Math.abs(targetSpeed) > 0.01);
    }

    public void setVoltage(double volts) {
        targetVoltage = Math.max(-12.0, Math.min(12.0, volts));
        targetSpeed = 0.0;
        motor.setControl(voltageRequest.withOutput(targetVoltage));

        SmartDashboard.putNumber(name + "/TargetVoltage", targetVoltage);
        SmartDashboard.putBoolean(name + "/Active", Math.abs(targetVoltage) > 0.1);
    }

    public void stop() {
        targetSpeed = 0.0;
        targetVoltage = 0.0;
        motor.setControl(dutyCycleRequest.withOutput(0));
        SmartDashboard.putBoolean(name + "/Active", false);
        SmartDashboard.putString(name + "/Status", "IDLE");
    }

    public double getTargetSpeed() { return targetSpeed; }
    public double getTargetVoltage() { return targetVoltage; }
    public double getVelocityRPS() { return velocitySignal.refresh().getValueAsDouble(); }
    public String getMotorName() { return name; }

    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        double velocity = velocitySignal.refresh().getValueAsDouble();
        double voltage  = voltageSignal.refresh().getValueAsDouble();
        double current  = currentSignal.refresh().getValueAsDouble();
        double temp     = tempSignal.refresh().getValueAsDouble();

        SmartDashboard.putNumber(name + "/TargetSpeed", targetSpeed);
        SmartDashboard.putNumber(name + "/TargetVoltage", targetVoltage);
        SmartDashboard.putBoolean(name + "/Active", Math.abs(targetSpeed) > 0.01 || Math.abs(targetVoltage) > 0.1);
        SmartDashboard.putNumber(name + "/VelocityRPS", Math.round(velocity * 100.0) / 100.0);
        SmartDashboard.putNumber(name + "/Voltage", Math.round(voltage * 100.0) / 100.0);
        SmartDashboard.putNumber(name + "/CurrentAmps", Math.round(current * 100.0) / 100.0);
        SmartDashboard.putNumber(name + "/TempC", Math.round(temp * 10.0) / 10.0);

        if (Math.abs(targetSpeed) > 0.01) {
            SmartDashboard.putString(name + "/Status", "DutyCycle " + String.format("%.2f", targetSpeed));
        } else if (Math.abs(targetVoltage) > 0.1) {
            SmartDashboard.putString(name + "/Status", "Voltage " + String.format("%.1fV", targetVoltage));
        } else {
            SmartDashboard.putString(name + "/Status", "IDLE");
        }
    }
}
