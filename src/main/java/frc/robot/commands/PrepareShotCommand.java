package frc.robot.commands;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * PREPARE SHOT COMMAND - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * Limelight + Odometry ile hub'a mesafe olcup, mesafeye gore:
 *   1) Shooter RPM'ini ayarlar (VelocityVoltage PID)
 *   2) Hood servo pozisyonunu ayarlar (atisacisi)
 *
 * Her ikisi de InterpolatingTreeMap ile mesafe bazli enterpolasyon yapar.
 * WCP referans degerleri temel alinmistir - sahada kalibre edilmeli!
 *
 * Mesafe Kaynaklari (oncelik sirasina gore):
 *   1) VisionSubsystem (Odometry + Vision Fusion) -> Hub mesafesi
 *   2) Limelight kamera (fallback) -> AprilTag mesafesi
 *
 * Kullanim:
 *   - Butona basili tutuldugu surece shooter + hood aktif
 *   - Birakilinca her ikisi de durur/default'a doner
 *   - 3 shooter motor + 2 hood servo birlikte calisir
 *
 * WCP Referans Tablo (mesafe inch -> RPM, hood pozisyonu):
 *   52"  (1.32m) -> 2800 RPM, hood 0.19
 *   81"  (2.06m) -> 3000 RPM, hood 0.32
 *   114" (2.90m) -> 3275 RPM, hood 0.40
 *   137" (3.48m) -> 3425 RPM, hood 0.43
 *   150" (3.81m) -> 3500 RPM, hood 0.46
 *   165" (4.19m) -> 3650 RPM, hood 0.48
 *   200" (5.08m) -> 4100 RPM, hood 0.55
 *   225" (5.72m) -> 4500 RPM, hood 0.60
 *   250" (6.35m) -> 5000 RPM, hood 0.65
 * ============================================================================
 */
public class PrepareShotCommand extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final VisionSubsystem vision;
    private final String limelightName;

    // ========================================================================
    // INTERPOLASYON TABLOLARI
    // Mesafe (metre) -> Deger
    // TreeMap kullanarak lineer enterpolasyon yapar
    // ========================================================================

    /** Mesafe (m) -> Shooter RPM */
    private static final TreeMap<Double, Double> RPM_TABLE = new TreeMap<>();

    /** Mesafe (m) -> Hood servo pozisyonu */
    private static final TreeMap<Double, Double> HOOD_TABLE = new TreeMap<>();

    static {
        // WCP referans degerleri (inch -> metreye cevrildi)
        // Yakin mesafe -> dusuk RPM, dusuk hood acisi
        // Uzak mesafe -> yuksek RPM, yuksek hood acisi
        RPM_TABLE.put(1.32, 2800.0);   //  52 inch
        RPM_TABLE.put(2.06, 3000.0);   //  81 inch
        RPM_TABLE.put(2.90, 3275.0);   // 114 inch
        RPM_TABLE.put(3.48, 3425.0);   // 137 inch
        RPM_TABLE.put(3.81, 3500.0);   // 150 inch
        RPM_TABLE.put(4.19, 3650.0);   // 165 inch
        RPM_TABLE.put(5.08, 4100.0);   // 200 inch
        RPM_TABLE.put(5.72, 4500.0);   // 225 inch
        RPM_TABLE.put(6.35, 5000.0);   // 250 inch

        HOOD_TABLE.put(1.32, 0.19);    //  52 inch
        HOOD_TABLE.put(2.06, 0.32);    //  81 inch
        HOOD_TABLE.put(2.90, 0.40);    // 114 inch
        HOOD_TABLE.put(3.48, 0.43);    // 137 inch
        HOOD_TABLE.put(3.81, 0.46);    // 150 inch
        HOOD_TABLE.put(4.19, 0.48);    // 165 inch
        HOOD_TABLE.put(5.08, 0.55);    // 200 inch
        HOOD_TABLE.put(5.72, 0.60);    // 225 inch
        HOOD_TABLE.put(6.35, 0.65);    // 250 inch
    }

    // ========================================================================
    // SABITLER
    // ========================================================================

    /** Minimum etkili mesafe (bunun altinda en yakin deger kullanilir) */
    private static final double MIN_DISTANCE = 1.0;

    /** Maksimum etkili mesafe (bunun ustunde motor calismaz) */
    private static final double MAX_DISTANCE = 7.0;

    /** Fallback RPM (mesafe bilinmiyorsa) */
    private static final double DEFAULT_RPM = 3200.0;

    /** Fallback hood pozisyonu (mesafe bilinmiyorsa) */
    private static final double DEFAULT_HOOD = 0.35;

    // ========================================================================
    // STATE
    // ========================================================================
    private double currentDistance = -1.0;
    private double currentRPM = 0.0;
    private double currentHoodPos = 0.0;
    private String distanceSource = "NONE";
    private boolean inRange = false;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 5;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Tam ozellikli constructor - Odometry + Limelight mesafe.
     *
     * @param shooter ShooterSubsystem (3 motor)
     * @param hood    HoodSubsystem (2 servo)
     * @param vision  VisionSubsystem (odometry bazli hub mesafesi)
     * @param limelightName Limelight ismi (fallback icin)
     */
    public PrepareShotCommand(ShooterSubsystem shooter, HoodSubsystem hood,
                              VisionSubsystem vision, String limelightName) {
        this.shooter = shooter;
        this.hood = hood;
        this.vision = vision;
        this.limelightName = limelightName;

        // Shooter ve Hood subsystem'lerini require et
        addRequirements(shooter, hood);
    }

    // ========================================================================
    // COMMAND LIFECYCLE
    // ========================================================================

    @Override
    public void initialize() {
        currentDistance = -1.0;
        currentRPM = 0.0;
        currentHoodPos = 0.0;
        distanceSource = "INITIALIZING";
        inRange = false;
        loopCount = 0;

        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    @Override
    public void execute() {
        loopCount++;

        // ==================================================================
        // ADIM 1: MESAFE HESAPLA
        // ==================================================================
        currentDistance = getHubDistance();

        // ==================================================================
        // ADIM 2: MENZIL KONTROLU
        // ==================================================================
        if (currentDistance < 0) {
            // Mesafe bilinmiyor -> fallback degerleri kullan
            currentRPM = DEFAULT_RPM;
            currentHoodPos = DEFAULT_HOOD;
            inRange = false;
            distanceSource = distanceSource.equals("NONE") ? "FALLBACK" : distanceSource + " (FALLBACK)";
        } else if (currentDistance > MAX_DISTANCE) {
            // Cok uzak -> motor calismaz
            currentRPM = 0.0;
            currentHoodPos = HoodSubsystem.DEFAULT_POSITION;
            inRange = false;
        } else {
            // Menzil icinde -> enterpolasyon
            currentRPM = interpolate(RPM_TABLE, currentDistance);
            currentHoodPos = interpolate(HOOD_TABLE, currentDistance);
            inRange = true;
        }

        // ==================================================================
        // ADIM 3: SHOOTER + HOOD KONTROL
        // ==================================================================
        if (currentRPM > 100) {
            shooter.setTargetRPM(currentRPM);
        } else {
            shooter.stop();
        }

        hood.setPosition(currentHoodPos);

        // ==================================================================
        // ADIM 4: DASHBOARD TELEMETRI
        // ==================================================================
        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putNumber("Shot/Distance", currentDistance > 0
                ? Math.round(currentDistance * 100.0) / 100.0 : -1);
            SmartDashboard.putNumber("Shot/TargetRPM", Math.round(currentRPM));
            SmartDashboard.putNumber("Shot/HoodPos", Math.round(currentHoodPos * 1000.0) / 1000.0);
            SmartDashboard.putString("Shot/Source", distanceSource);
            SmartDashboard.putBoolean("Shot/InRange", inRange);
            SmartDashboard.putBoolean("Shot/ShooterReady", shooter.atTargetSpeed());
            SmartDashboard.putBoolean("Shot/ReadyToFire",
                inRange && shooter.atTargetSpeed() && hood.atTarget());
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hood.setDefault();

        LimelightHelpers.setLEDMode_PipelineControl(limelightName);

        SmartDashboard.putNumber("Shot/TargetRPM", 0);
        SmartDashboard.putString("Shot/Source", "STOPPED");
        SmartDashboard.putBoolean("Shot/InRange", false);
        SmartDashboard.putBoolean("Shot/ReadyToFire", false);
    }

    @Override
    public boolean isFinished() {
        // whileTrue ile kullanilir - buton basili oldugu surece calisir
        return false;
    }

    // ========================================================================
    // MESAFE HESAPLAMA
    // ========================================================================

    /**
     * Hub'a olan mesafeyi hesaplar.
     * Oncelik 1: Odometry (VisionSubsystem)
     * Oncelik 2: Limelight kamera (fallback)
     */
    private double getHubDistance() {
        // --- ONCELIK 1: Odometry bazli hub mesafesi ---
        if (vision != null) {
            double odometryDist = vision.getDistanceToOwnHub();
            if (odometryDist >= 0 && odometryDist < 20.0) {
                distanceSource = "ODOMETRY";
                return odometryDist;
            }
        }

        // --- ONCELIK 2: Limelight kamera mesafesi ---
        distanceSource = "CAMERA";
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        if (hasTarget) {
            double[] cameraPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);

            if (cameraPose != null && cameraPose.length >= 3 && cameraPose[2] > 0.05) {
                double cx = cameraPose[0];
                double cz = cameraPose[2];
                double dist = Math.sqrt(cx * cx + cz * cz);

                if (dist > 0.1 && dist < 10.0) {
                    return dist;
                }
            }
        }

        distanceSource = "NONE";
        return -1.0;
    }

    // ========================================================================
    // LINEER ENTERPOLASYON
    // ========================================================================

    /**
     * TreeMap uzerinden lineer enterpolasyon yapar.
     * Tablodaki iki nokta arasinda kalan mesafeler icin dogusal hesap yapar.
     * Tablonun altindaki degerler ilk noktayi, ustundekiler son noktayi kullanir.
     *
     * @param table TreeMap (mesafe -> deger)
     * @param key   Mesafe (metre)
     * @return Enterpolasyon sonucu deger
     */
    private static double interpolate(TreeMap<Double, Double> table, double key) {
        // Tam eslesme varsa direkt don
        Double exact = table.get(key);
        if (exact != null) return exact;

        // Alt ve ust sinir
        Double lowerKey = table.floorKey(key);
        Double upperKey = table.ceilingKey(key);

        // Tablonun disinda
        if (lowerKey == null) return table.firstEntry().getValue();
        if (upperKey == null) return table.lastEntry().getValue();

        // Lineer enterpolasyon
        double lowerVal = table.get(lowerKey);
        double upperVal = table.get(upperKey);
        double ratio = (key - lowerKey) / (upperKey - lowerKey);

        return lowerVal + ratio * (upperVal - lowerVal);
    }

    // ========================================================================
    // PUBLIC GETTER'LAR
    // ========================================================================

    /** Suanki hub mesafesi (metre) */
    public double getCurrentDistance() { return currentDistance; }

    /** Suanki hedef RPM */
    public double getCurrentRPM() { return currentRPM; }

    /** Suanki hood pozisyonu */
    public double getCurrentHoodPos() { return currentHoodPos; }

    /** Menzil icinde mi? */
    public boolean isInRange() { return inRange; }

    /**
     * Atisa hazir mi?
     * Menzil icinde + shooter hedef hizda + hood hedef pozisyonda
     */
    public boolean isReadyToFire() {
        return inRange && shooter.atTargetSpeed() && hood.atTarget();
    }
}
