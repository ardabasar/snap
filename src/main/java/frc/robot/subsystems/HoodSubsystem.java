package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * HOOD SUBSYSTEM - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * 2 adet PWM Servo ile shooter atisacisini (hood) ayarlar.
 * WCP referans: Mesafeye gore servo pozisyonu InterpolatingTreeMap ile belirlenir.
 *
 * Donanim:
 *   - Sol Servo:  PWM 3 (roboRIO)
 *   - Sag Servo:  PWM 4 (roboRIO)
 *   - Her iki servo ayni pozisyona gider
 *
 * Kontrol:
 *   - PrepareShotCommand mesafeye gore setPosition() cagirir
 *   - Servo pozisyonu: 0.0 (en dusuk aci) - 1.0 (en yuksek aci)
 *   - WCP referans aralik: 0.0 - 0.77
 *
 * WCP Referans Hood Acisi Tablosu (mesafe -> servo pozisyonu):
 *   52 inch  (1.32m) -> 0.19
 *   81 inch  (2.06m) -> 0.32
 *   114 inch (2.90m) -> 0.40
 *   150 inch (3.81m) -> 0.46
 *   165 inch (4.19m) -> 0.48
 *   200 inch (5.08m) -> 0.55
 *   225 inch (5.72m) -> 0.60
 *   250 inch (6.35m) -> 0.65
 *
 * NOT: Bu degerler WCP Big Dumper icin kalibre edilmistir.
 * Sizin robotunuz icin sahada test ile ayarlanmali!
 * ============================================================================
 */
public class HoodSubsystem extends SubsystemBase {

    // ========================================================================
    // PWM PORTLARI
    // ========================================================================
    public static final int LEFT_SERVO_PWM  = 3;
    public static final int RIGHT_SERVO_PWM = 4;

    // ========================================================================
    // SABITLER
    // ========================================================================

    /** Servo minimum pozisyonu (en dusuk aci) */
    public static final double MIN_POSITION = 0.0;

    /** Servo maximum pozisyonu (en yuksek aci) */
    public static final double MAX_POSITION = 0.77;

    /** Varsayilan pozisyon (baslangic/idle) */
    public static final double DEFAULT_POSITION = 0.0;

    /** Pozisyon toleransi */
    private static final double POSITION_TOLERANCE = 0.02;

    // ========================================================================
    // DONANIM
    // ========================================================================
    private final Servo leftServo;
    private final Servo rightServo;

    // ========================================================================
    // STATE
    // ========================================================================
    private double targetPosition = DEFAULT_POSITION;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public HoodSubsystem() {
        leftServo  = new Servo(LEFT_SERVO_PWM);
        rightServo = new Servo(RIGHT_SERVO_PWM);

        // Baslangic pozisyonu
        setPosition(DEFAULT_POSITION);

        System.out.println("[Hood] Initialized - PWM " + LEFT_SERVO_PWM + ", " + RIGHT_SERVO_PWM);
    }

    // ========================================================================
    // KONTROL METODLARI
    // ========================================================================

    /**
     * Hood'u belirtilen pozisyona ayarlar.
     * @param position 0.0 (dusuk aci) - MAX_POSITION (yuksek aci)
     */
    public void setPosition(double position) {
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));
        leftServo.set(targetPosition);
        rightServo.set(targetPosition);
    }

    /**
     * Hood'u baslangic/idle pozisyonuna getirir.
     */
    public void setDefault() {
        setPosition(DEFAULT_POSITION);
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================

    /** Hedef servo pozisyonu */
    public double getTargetPosition() {
        return targetPosition;
    }

    /** Sol servo gercek pozisyonu */
    public double getLeftPosition() {
        return leftServo.get();
    }

    /** Sag servo gercek pozisyonu */
    public double getRightPosition() {
        return rightServo.get();
    }

    /**
     * Hood hedef pozisyona ulasti mi?
     * (Servo'lar aninda gitmez, kucuk bir gecikme olabilir)
     */
    public boolean atTarget() {
        return Math.abs(getLeftPosition() - targetPosition) < POSITION_TOLERANCE;
    }

    // ========================================================================
    // PERIODIC - Telemetri
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("Hood/TargetPos", Math.round(targetPosition * 1000.0) / 1000.0);
        SmartDashboard.putNumber("Hood/LeftPos", Math.round(getLeftPosition() * 1000.0) / 1000.0);
        SmartDashboard.putNumber("Hood/RightPos", Math.round(getRightPosition() * 1000.0) / 1000.0);
        SmartDashboard.putBoolean("Hood/AtTarget", atTarget());
    }
}
