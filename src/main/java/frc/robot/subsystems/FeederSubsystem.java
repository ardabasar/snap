package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * FEEDER SUBSYSTEM - Snapstrike 2026 REBUILT
 * ============================================================================
 * 1 adet TalonFX (Kraken X60) ile surulen feeder mekanizmasi.
 * Toplari shooter'a besler.
 *
 * Donanim:
 *   - Motor: CAN 16 (rio bus)
 *   - Brake mode (top tutma icin)
 *
 * Kontrol:
 *   - DutyCycle 0.25 hizinda (basili tutuldugu surece)
 *   - Buton birakilinca otomatik durur
 *
 * WCP Referans: Feeder VelocityVoltage ile 5000 RPM kullanir.
 * Biz basit DutyCycle ile basliyoruz, ileride PID'e upgrade edilebilir.
 * ============================================================================
 */
public class FeederSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID
    // ========================================================================
    public static final int MOTOR_CAN_ID = 16;
    public static final String CAN_BUS   = "rio";

    // ========================================================================
    // SABITLER
    // ========================================================================
    /** Varsayilan besleme hizi (DutyCycle: 0.0-1.0) */
    public static final double DEFAULT_SPEED = 0.25;

    /** Stator akim limiti (Amper) */
    private static final double STATOR_CURRENT_LIMIT = 40.0;

    // ========================================================================
    // DONANIM
    // ========================================================================
    private final TalonFX motor;
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // ========================================================================
    // TELEMETRI SINYALLERI
    // ========================================================================
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> currentSignal;

    // ========================================================================
    // STATE
    // ========================================================================
    private double targetSpeed = 0.0;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public FeederSubsystem() {
        motor = new TalonFX(MOTOR_CAN_ID, CAN_BUS);

        configureMotor();

        velocitySignal = motor.getRotorVelocity();
        currentSignal  = motor.getStatorCurrent();

        stop();
        System.out.println("[Feeder] Initialized - CAN " + MOTOR_CAN_ID);
    }

    // ========================================================================
    // MOTOR KONFIGURASYONU
    // ========================================================================
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode - top tutma
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Akim limiti
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

        motor.getConfigurator().apply(config);
    }

    // ========================================================================
    // KONTROL METODLARI
    // ========================================================================

    /**
     * Feeder'i belirtilen hizda calistirir.
     * @param speed -1.0 ile 1.0 arasi DutyCycle
     */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        motor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /**
     * Feeder'i varsayilan hizda ileri calistirir (toplari shooter'a besler).
     */
    public void feed() {
        setSpeed(DEFAULT_SPEED);
    }

    /**
     * Feeder'i ters calistirir (top cikarma).
     */
    public void reverse() {
        setSpeed(-DEFAULT_SPEED);
    }

    /**
     * Feeder'i durdurur.
     */
    public void stop() {
        targetSpeed = 0.0;
        motor.setControl(dutyCycleRequest.withOutput(0));
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================

    public double getVelocityRPS() {
        return velocitySignal.refresh().getValueAsDouble();
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public boolean isActive() {
        return Math.abs(targetSpeed) > 0.01;
    }

    // ========================================================================
    // PERIODIC - Telemetri
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("Feeder/VelocityRPS", Math.round(getVelocityRPS() * 100.0) / 100.0);
        SmartDashboard.putNumber("Feeder/Current", Math.round(currentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putNumber("Feeder/TargetSpeed", targetSpeed);
        SmartDashboard.putBoolean("Feeder/Active", isActive());
    }
}
