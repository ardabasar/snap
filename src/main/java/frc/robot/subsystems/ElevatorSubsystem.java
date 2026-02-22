package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * ELEVATOR SUBSYSTEM - Snapstrike 2026 REBUILT
 * ============================================================================
 * 2 adet TalonFX (Kraken X60) ile surulen elevator mekanizmasi.
 * Leader-Follower konfigurasyonu kullanir.
 *
 * Donanim:
 *   - Leader Motor:   CAN 9  (rio bus)
 *   - Follower Motor: CAN 10 (rio bus)
 *   - Follower, leader'i ters cevirilmeden takip eder (ayni yone doner)
 *   - Brake mode aktif (elevator yukari kaldirmada geri dusmemesi icin)
 *
 * Kontrol:
 *   - Joystick ile 0.25 DutyCycle hizinda (basili tutuldugu surece)
 *   - Buton birakilinca otomatik durur (Brake mode tutar)
 *
 * Guvenlik:
 *   - Akim limiti: 40A (elevator mekanizmasi icin)
 *   - Soft limit'ler mekanik montajdan sonra kalibre edilebilir
 * ============================================================================
 */
public class ElevatorSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID'LER
    // ========================================================================
    public static final int LEADER_CAN_ID   = 9;
    public static final int FOLLOWER_CAN_ID = 10;
    public static final String CAN_BUS      = "rio";

    // ========================================================================
    // SABITLER
    // ========================================================================
    /** Varsayilan hareket hizi (DutyCycle: 0.0-1.0) */
    public static final double DEFAULT_SPEED = 0.25;

    /** Stator akim limiti (Amper) */
    private static final double STATOR_CURRENT_LIMIT = 40.0;

    /** Follower motor hizalamasi (Aligned = leader ile ayni yon) */
    private static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Aligned;

    // ========================================================================
    // DONANIM
    // ========================================================================
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;

    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // ========================================================================
    // TELEMETRI SINYALLERI
    // ========================================================================
    private final StatusSignal<Angle> positionSignal;
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
    public ElevatorSubsystem() {
        leaderMotor   = new TalonFX(LEADER_CAN_ID, CAN_BUS);
        followerMotor = new TalonFX(FOLLOWER_CAN_ID, CAN_BUS);

        configureMotors();

        // Follower, leader'i takip etsin
        followerMotor.setControl(new Follower(LEADER_CAN_ID, FOLLOWER_ALIGNMENT));

        // Telemetri sinyalleri (sadece leader'dan)
        positionSignal = leaderMotor.getRotorPosition();
        velocitySignal = leaderMotor.getRotorVelocity();
        currentSignal  = leaderMotor.getStatorCurrent();

        stop();
        System.out.println("[Elevator] Initialized - Leader CAN " + LEADER_CAN_ID
            + ", Follower CAN " + FOLLOWER_CAN_ID);
    }

    // ========================================================================
    // MOTOR KONFIGURASYONU
    // ========================================================================
    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode - elevator dusmesin
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Akim limiti
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

        // Leader konfigurasyonu
        leaderMotor.getConfigurator().apply(config);

        // Follower icin ayni config
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

        followerMotor.getConfigurator().apply(followerConfig);
    }

    // ========================================================================
    // KONTROL METODLARI
    // ========================================================================

    /**
     * Elevator'u belirtilen hizda hareket ettirir.
     * @param speed -1.0 (asagi) ile 1.0 (yukari) arasi DutyCycle
     */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        leaderMotor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /**
     * Elevator'u varsayilan hizda yukari kaldirir.
     */
    public void moveUp() {
        setSpeed(DEFAULT_SPEED);
    }

    /**
     * Elevator'u varsayilan hizda asagi indirir.
     */
    public void moveDown() {
        setSpeed(-DEFAULT_SPEED);
    }

    /**
     * Elevator'u durdurur (Brake mode aktif, pozisyonda kalir).
     */
    public void stop() {
        targetSpeed = 0.0;
        leaderMotor.setControl(dutyCycleRequest.withOutput(0));
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================

    /** Leader motor pozisyonu (rotor donusu) */
    public double getPositionRotations() {
        return positionSignal.refresh().getValueAsDouble();
    }

    /** Leader motor hizi (rotor donusu/saniye) */
    public double getVelocityRPS() {
        return velocitySignal.refresh().getValueAsDouble();
    }

    /** Hedef hiz */
    public double getTargetSpeed() {
        return targetSpeed;
    }

    /** Motor aktif mi? */
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

        SmartDashboard.putNumber("Elevator/Position", Math.round(getPositionRotations() * 100.0) / 100.0);
        SmartDashboard.putNumber("Elevator/VelocityRPS", Math.round(getVelocityRPS() * 100.0) / 100.0);
        SmartDashboard.putNumber("Elevator/Current", Math.round(currentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putNumber("Elevator/TargetSpeed", targetSpeed);
        SmartDashboard.putBoolean("Elevator/Active", isActive());
    }
}
