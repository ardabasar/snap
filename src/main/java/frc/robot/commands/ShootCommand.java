package frc.robot.commands;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * SHOOT COMMAND - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * Ana atis komutu. Limelight ile mesafe olcup, WCP referans tablosundan
 * enterpolasyon ile Shooter RPM ve Hood acisini ayarlar.
 *
 * ONEMLI: Feeder, shooter hedef hiza ulastiktan SONRA baslar!
 * Bu sayede top, shooter hazir olmadan atilmaz.
 *
 * Calisma Adimlari:
 *   1) Limelight LED'leri ac
 *   2) Hub'a mesafe olc (Odometry -> Limelight fallback)
 *   3) Mesafeye gore RPM ve hood acisi hesapla (enterpolasyon)
 *   4) Shooter motorlarini hedef RPM'e calistir
 *   5) Hood servolarini hedef aciya ayarla
 *   6) BEKLE: Shooter hedef RPM'e ulassin (atTargetSpeed)
 *   7) Shooter hazir -> Feeder'i baslat (0.25 hiz)
 *   8) Buton birakilinca: Hepsi durur, hood sifira doner
 *
 * Fark (eski PrepareShotCommand'den):
 *   - Eski: Shooter + Feeder ayni anda baslar -> top hazir olmadan gidebilir
 *   - Yeni: Shooter ONCE hiza ulasir -> SONRA feeder baslar -> tutarli atis
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
 *
 * NOT: Bu degerler sahada kalibre edilmelidir!
 * ============================================================================
 */
public class ShootCommand extends Command {

    // ========================================================================
    // SUBSYSTEM'LER
    // ========================================================================
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final VisionSubsystem vision;
    private final String limelightName;

    // ========================================================================
    // INTERPOLASYON TABLOLARI
    // Mesafe (metre) -> Deger
    // ========================================================================

    /** Mesafe (m) -> Shooter RPM */
    private static final TreeMap<Double, Double> RPM_TABLE = new TreeMap<>();

    /** Mesafe (m) -> Hood servo pozisyonu */
    private static final TreeMap<Double, Double> HOOD_TABLE = new TreeMap<>();

    static {
        // WCP referans degerleri (inch -> metreye cevrildi)
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
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 7.0;
    private static final double DEFAULT_RPM = 3200.0;
    private static final double DEFAULT_HOOD = 0.35;

    // ========================================================================
    // STATE
    // ========================================================================

    /** Suanki atis durumu */
    private enum ShootState {
        /** Mesafe olculuyor ve shooter hizlaniyor */
        SPINNING_UP,
        /** Shooter hazir, feeder basladi - atis yapiliyor */
        FIRING,
        /** Menzil disi veya mesafe bilinmiyor */
        OUT_OF_RANGE
    }

    private ShootState state = ShootState.SPINNING_UP;
    private double currentDistance = -1.0;
    private double currentRPM = 0.0;
    private double currentHoodPos = 0.0;
    private String distanceSource = "NONE";

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 5;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Atis komutunu olusturur.
     *
     * @param shooter  ShooterSubsystem (3 motor)
     * @param hood     HoodSubsystem (2 servo)
     * @param feeder   FeederSubsystem (1 motor)
     * @param vision   VisionSubsystem (odometry bazli mesafe)
     * @param limelightName Limelight ismi
     */
    public ShootCommand(ShooterSubsystem shooter, HoodSubsystem hood,
                        FeederSubsystem feeder, VisionSubsystem vision,
                        String limelightName) {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.vision = vision;
        this.limelightName = limelightName;

        // 3 subsystem birden require edilir
        addRequirements(shooter, hood, feeder);
    }

    // ========================================================================
    // COMMAND LIFECYCLE
    // ========================================================================

    @Override
    public void initialize() {
        state = ShootState.SPINNING_UP;
        currentDistance = -1.0;
        currentRPM = 0.0;
        currentHoodPos = 0.0;
        distanceSource = "INITIALIZING";
        loopCount = 0;

        // Limelight LED'lerini ac
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
        // ADIM 2: RPM ve HOOD HESAPLA
        // ==================================================================
        if (currentDistance < 0) {
            // Mesafe bilinmiyor -> fallback degerleri
            currentRPM = DEFAULT_RPM;
            currentHoodPos = DEFAULT_HOOD;
        } else if (currentDistance > MAX_DISTANCE) {
            // Cok uzak -> motor calismaz
            currentRPM = 0.0;
            currentHoodPos = HoodSubsystem.DEFAULT_POSITION;
            state = ShootState.OUT_OF_RANGE;
        } else {
            // Menzil icinde -> enterpolasyon
            currentRPM = interpolate(RPM_TABLE, currentDistance);
            currentHoodPos = interpolate(HOOD_TABLE, currentDistance);
        }

        // ==================================================================
        // ADIM 3: SHOOTER + HOOD KONTROL (her zaman calisir)
        // ==================================================================
        if (currentRPM > 100) {
            shooter.setTargetRPM(currentRPM);
            hood.setPosition(currentHoodPos);
        } else {
            shooter.stop();
            feeder.stop();
            hood.setDefault();
            state = ShootState.OUT_OF_RANGE;
        }

        // ==================================================================
        // ADIM 4: STATE MAKINESINI GUNCELLE
        // ==================================================================
        if (state != ShootState.OUT_OF_RANGE) {
            if (shooter.atTargetSpeed()) {
                // Shooter hazir -> Feeder'i baslat!
                state = ShootState.FIRING;
                feeder.feed();
            } else {
                // Shooter henuz hizlanmadi -> Feeder'i DURDUR
                state = ShootState.SPINNING_UP;
                feeder.stop();
            }
        }

        // ==================================================================
        // ADIM 5: DASHBOARD TELEMETRI
        // ==================================================================
        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putString("Shoot/State", state.name());
            SmartDashboard.putNumber("Shoot/Distance",
                currentDistance > 0 ? Math.round(currentDistance * 100.0) / 100.0 : -1);
            SmartDashboard.putNumber("Shoot/TargetRPM", Math.round(currentRPM));
            SmartDashboard.putNumber("Shoot/ActualRPM", Math.round(shooter.getAverageRPM()));
            SmartDashboard.putNumber("Shoot/HoodPos",
                Math.round(currentHoodPos * 1000.0) / 1000.0);
            SmartDashboard.putString("Shoot/Source", distanceSource);
            SmartDashboard.putBoolean("Shoot/ShooterReady", shooter.atTargetSpeed());
            SmartDashboard.putBoolean("Shoot/Feeding", state == ShootState.FIRING);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Hepsini durdur
        shooter.stop();
        feeder.stop();
        hood.setDefault();  // Servo sifira doner

        // Limelight LED pipeline kontrolune donsun
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);

        // Dashboard temizle
        SmartDashboard.putString("Shoot/State", "STOPPED");
        SmartDashboard.putNumber("Shoot/TargetRPM", 0);
        SmartDashboard.putBoolean("Shoot/ShooterReady", false);
        SmartDashboard.putBoolean("Shoot/Feeding", false);
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
     * Oncelik 1: Odometry (VisionSubsystem) - daha guvenilir
     * Oncelik 2: Limelight kamera (fallback)
     *
     * @return Mesafe (metre), bulunamazsa -1.0
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
     *
     * Ornek: Mesafe 2.5m ise, tabloda 2.06m (3000 RPM) ve 2.90m (3275 RPM)
     * arasinda hesap yapilir:
     *   ratio = (2.5 - 2.06) / (2.90 - 2.06) = 0.524
     *   RPM = 3000 + 0.524 * (3275 - 3000) = 3144 RPM
     *
     * @param table Mesafe -> deger tablosu
     * @param key   Mesafe (metre)
     * @return Enterpolasyon sonucu
     */
    private static double interpolate(TreeMap<Double, Double> table, double key) {
        Double exact = table.get(key);
        if (exact != null) return exact;

        Double lowerKey = table.floorKey(key);
        Double upperKey = table.ceilingKey(key);

        if (lowerKey == null) return table.firstEntry().getValue();
        if (upperKey == null) return table.lastEntry().getValue();

        double lowerVal = table.get(lowerKey);
        double upperVal = table.get(upperKey);
        double ratio = (key - lowerKey) / (upperKey - lowerKey);

        return lowerVal + ratio * (upperVal - lowerVal);
    }

    // ========================================================================
    // PUBLIC GETTER'LAR (Dashboard / debug icin)
    // ========================================================================

    public double getCurrentDistance() { return currentDistance; }
    public double getCurrentRPM() { return currentRPM; }
    public double getCurrentHoodPos() { return currentHoodPos; }
    public ShootState getState() { return state; }
    public boolean isFiring() { return state == ShootState.FIRING; }
}
