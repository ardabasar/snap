package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * STRUCTURE SUBSYSTEM - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * Topları shooter'a besleyen alt mekanizma (conveyor/bucket tilt).
 * 2 adet TalonFX (Kraken X60) ile leader-follower konfigurasyonu.
 *
 * Donanim:
 *   - Leader Motor:   CAN 11 (rio bus)
 *   - Follower Motor: CAN 12 (rio bus)
 *   - Brake mode aktif
 *
 * Kontrol:
 *   - Joystick ile 0.25 DutyCycle hizinda (basili tutuldugu surece)
 *   - Buton birakilinca otomatik durur
 * ============================================================================
 */
public class StructureSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID'LER
    // ========================================================================
    public static final int LEADER_CAN_ID   = 11;
    public static final int FOLLOWER_CAN_ID = 12;
    public static final String CAN_BUS      = "rio";

    // ========================================================================
    // SABITLER
    // ========================================================================
    /** Varsayilan hareket hizi (DutyCycle: 0.0-1.0) */
    public static final double DEFAULT_SPEED = 0.25;

    /** Stator akim limiti (Amper) */
    private static final double STATOR_CURRENT_LIMIT = 40.0;

    /** Follower motor yonunu ters cevir? */
    private static final boolean FOLLOWER_OPPOSE_LEADER = false;

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
    public StructureSubsystem() {
        leaderMotor   = new TalonFX(LEADER_CAN_ID, CAN_BUS);
        followerMotor = new TalonFX(FOLLOWER_CAN_ID, CAN_BUS);

        configureMotors();

        // Follower, leader'i takip etsin
        followerMotor.setControl(new Follower(LEADER_CAN_ID, FOLLOWER_OPPOSE_LEADER));

        // Telemetri sinyalleri (sadece leader'dan)
        positionSignal = leaderMotor.getRotorPosition();
        velocitySignal = leaderMotor.getRotorVelocity();
        currentSignal  = leaderMotor.getStatorCurrent();

        stop();
        System.out.println("[Structure] Initialized - Leader CAN " + LEADER_CAN_ID
            + ", Follower CAN " + FOLLOWER_CAN_ID);
    }

    // ========================================================================
    // MOTOR KONFIGURASYONU
    // ========================================================================
    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Akim limiti
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

        leaderMotor.getConfigurator().apply(config);

        // Follower config
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
     * Structure'i belirtilen hizda hareket ettirir.
     * @param speed -1.0 ile 1.0 arasi DutyCycle
     */
    public void setSpeed(double speed) {
        targetSpeed = Math.max(-1.0, Math.min(1.0, speed));
        leaderMotor.setControl(dutyCycleRequest.withOutput(targetSpeed));
    }

    /**
     * Structure'i varsayilan hizda ileri calistirir (topları yukari besler).
     */
    public void feedForward() {
        setSpeed(DEFAULT_SPEED);
    }

    /**
     * Structure'i varsayilan hizda geri calistirir.
     */
    public void feedReverse() {
        setSpeed(-DEFAULT_SPEED);
    }

    /**
     * Structure'i durdurur.
     */
    public void stop() {
        targetSpeed = 0.0;
        leaderMotor.setControl(dutyCycleRequest.withOutput(0));
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================

    public double getPositionRotations() {
        return positionSignal.refresh().getValueAsDouble();
    }

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

        SmartDashboard.putNumber("Structure/Position", Math.round(getPositionRotations() * 100.0) / 100.0);
        SmartDashboard.putNumber("Structure/VelocityRPS", Math.round(getVelocityRPS() * 100.0) / 100.0);
        SmartDashboard.putNumber("Structure/Current", Math.round(currentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putNumber("Structure/TargetSpeed", targetSpeed);
        SmartDashboard.putBoolean("Structure/Active", isActive());
    }
}
