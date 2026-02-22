package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * SHOOTER SUBSYSTEM - Snapstrike 2026 REBUILT
 * ============================================================================
 * 3 adet TalonFX (Kraken X60) ile surulen shooter mekanizmasi.
 * WCP referans: VelocityVoltage PID ile RPM kontrolu.
 *
 * Donanim:
 *   - Sol Motor:   CAN 13 (rio bus)
 *   - Orta Motor:  CAN 14 (rio bus)
 *   - Sag Motor:   CAN 15 (rio bus)
 *   - 3 motor BAGIMSIZ calisir (hepsi ayni RPM hedefine gider)
 *   - Coast mode (shooter serbest donebilmeli)
 *
 * Kontrol:
 *   - VelocityVoltage: PID ile hedef RPM'e ulasir
 *   - Mesafe bazli RPM: PrepareShotCommand tarafindan set edilir
 *   - Tum motorlar ayni anda ayni RPM'de doner
 *
 * PID (WCP referans degerleri, kalibre edilmeli):
 *   - kP = 0.2
 *   - kI = 0.0
 *   - kD = 0.0
 *   - kV = 0.12 (feedforward: 1/RPSmax, Kraken ~100 RPS = 6000 RPM)
 * ============================================================================
 */
public class ShooterSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID'LER
    // ========================================================================
    public static final int LEFT_CAN_ID   = 13;
    public static final int MIDDLE_CAN_ID = 14;
    public static final int RIGHT_CAN_ID  = 15;
    public static final String CAN_BUS    = "rio";

    // ========================================================================
    // PID SABITLERI (VelocityVoltage Slot 0)
    // WCP referans + Kraken X60 icin optimize
    // ========================================================================
    private static final double kP = 0.2;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.12;   // Feedforward (Volt / RPS)
    private static final double kS = 0.05;   // Static friction compensation

    /** Stator akim limiti (Amper) - Shooter yuksek akim ceker */
    private static final double STATOR_CURRENT_LIMIT = 60.0;

    /** RPM toleransi - hedefe bu kadar yaklasinca "atHiz" sayilir */
    private static final double RPM_TOLERANCE = 150.0;

    // ========================================================================
    // DONANIM
    // ========================================================================
    private final TalonFX leftMotor;
    private final TalonFX middleMotor;
    private final TalonFX rightMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final DutyCycleOut stopRequest = new DutyCycleOut(0);

    // ========================================================================
    // TELEMETRI SINYALLERI (sol motordan referans)
    // ========================================================================
    private final StatusSignal<AngularVelocity> leftVelocitySignal;
    private final StatusSignal<AngularVelocity> middleVelocitySignal;
    private final StatusSignal<AngularVelocity> rightVelocitySignal;
    private final StatusSignal<Current> leftCurrentSignal;

    // ========================================================================
    // STATE
    // ========================================================================
    private double targetRPM = 0.0;
    private boolean running = false;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public ShooterSubsystem() {
        leftMotor   = new TalonFX(LEFT_CAN_ID, CAN_BUS);
        middleMotor = new TalonFX(MIDDLE_CAN_ID, CAN_BUS);
        rightMotor  = new TalonFX(RIGHT_CAN_ID, CAN_BUS);

        configureMotors();

        // Telemetri sinyalleri
        leftVelocitySignal   = leftMotor.getRotorVelocity();
        middleVelocitySignal = middleMotor.getRotorVelocity();
        rightVelocitySignal  = rightMotor.getRotorVelocity();
        leftCurrentSignal    = leftMotor.getStatorCurrent();

        stop();
        System.out.println("[Shooter] Initialized - CAN " + LEFT_CAN_ID
            + ", " + MIDDLE_CAN_ID + ", " + RIGHT_CAN_ID);
    }

    // ========================================================================
    // MOTOR KONFIGURASYONU
    // ========================================================================
    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Coast mode - shooter serbest donebilmeli
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // PID Slot 0 - VelocityVoltage icin
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kS = kS;

        // Akim limiti
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

        // Sol motor (referans yon)
        leftMotor.getConfigurator().apply(config);

        // Orta motor (ayni yon)
        middleMotor.getConfigurator().apply(config);

        // Sag motor (ters yon olabilir - mekanik montaja bagli)
        // Eger sag motor ters donuyorsa asagidaki satiri aktif et:
        // config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(config);
    }

    // ========================================================================
    // KONTROL METODLARI
    // ========================================================================

    /**
     * Shooter'i hedef RPM'de calistirir (3 motor birlikte).
     * VelocityVoltage PID kullanir.
     *
     * @param rpm Hedef RPM (pozitif = atma yonu)
     */
    public void setTargetRPM(double rpm) {
        targetRPM = Math.abs(rpm);
        running = true;

        // RPM -> RPS cevirimi (TalonFX rotations/second cinsinden calisir)
        double targetRPS = targetRPM / 60.0;

        leftMotor.setControl(velocityRequest.withVelocity(targetRPS));
        middleMotor.setControl(velocityRequest.withVelocity(targetRPS));
        rightMotor.setControl(velocityRequest.withVelocity(targetRPS));
    }

    /**
     * Shooter'i durdurur (Coast - serbest yavaslar).
     */
    public void stop() {
        targetRPM = 0.0;
        running = false;
        leftMotor.setControl(stopRequest.withOutput(0));
        middleMotor.setControl(stopRequest.withOutput(0));
        rightMotor.setControl(stopRequest.withOutput(0));
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================

    /** Sol motor gercek RPM */
    public double getLeftRPM() {
        return leftVelocitySignal.refresh().getValueAsDouble() * 60.0;
    }

    /** Orta motor gercek RPM */
    public double getMiddleRPM() {
        return middleVelocitySignal.refresh().getValueAsDouble() * 60.0;
    }

    /** Sag motor gercek RPM */
    public double getRightRPM() {
        return rightVelocitySignal.refresh().getValueAsDouble() * 60.0;
    }

    /** 3 motorun ortalama RPM'i */
    public double getAverageRPM() {
        return (getLeftRPM() + getMiddleRPM() + getRightRPM()) / 3.0;
    }

    /** Hedef RPM */
    public double getTargetRPM() {
        return targetRPM;
    }

    /** Shooter calisiyor mu? */
    public boolean isRunning() {
        return running;
    }

    /**
     * Shooter hedef hiza ulasti mi?
     * Tum motorlar tolerans icinde olmali.
     */
    public boolean atTargetSpeed() {
        if (!running || targetRPM < 100) return false;

        double leftErr   = Math.abs(getLeftRPM() - targetRPM);
        double middleErr = Math.abs(getMiddleRPM() - targetRPM);
        double rightErr  = Math.abs(getRightRPM() - targetRPM);

        return leftErr < RPM_TOLERANCE
            && middleErr < RPM_TOLERANCE
            && rightErr < RPM_TOLERANCE;
    }

    // ========================================================================
    // PERIODIC - Telemetri
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        double avgRPM = getAverageRPM();

        SmartDashboard.putNumber("Shooter/TargetRPM", Math.round(targetRPM));
        SmartDashboard.putNumber("Shooter/AvgRPM", Math.round(avgRPM));
        SmartDashboard.putNumber("Shooter/LeftRPM", Math.round(getLeftRPM()));
        SmartDashboard.putNumber("Shooter/MiddleRPM", Math.round(getMiddleRPM()));
        SmartDashboard.putNumber("Shooter/RightRPM", Math.round(getRightRPM()));
        SmartDashboard.putNumber("Shooter/Current", Math.round(leftCurrentSignal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putBoolean("Shooter/Running", running);
        SmartDashboard.putBoolean("Shooter/AtSpeed", atTargetSpeed());

        if (running) {
            double error = Math.abs(avgRPM - targetRPM);
            SmartDashboard.putNumber("Shooter/ErrorRPM", Math.round(error));
        }
    }
}
