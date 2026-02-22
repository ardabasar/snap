package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.KrakenMotorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * DISTANCE-BASED DUMPER COMMAND - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * FRC 2026 REBUILT sahasinda Hub'a fuel skorlamak icin:
 * 
 * Konum Kaynagi (oncelik sirasina gore):
 *   1) ODOMETRY + VISION FUSION (VisionSubsystem uzerinden)
 *      -> drivetrain.getState().Pose ile robotun saha uzerindeki
 *         gercek X,Y koordinatini alir
 *      -> Vision aktifse AprilTag ile duzeltilmis, degilse pure odometry
 *      -> FMS'e baglaninca DriverStation.getAlliance() ile dogru Hub secilir
 *   
 *   2) LIMELIGHT KAMERA (fallback)
 *      -> AprilTag gorunuyorsa kameradan mesafe hesaplar
 *      -> Odometry mevcut degilse veya VisionSubsystem yoksa kullanilir
 * 
 * Voltaj Profili (Big Dumper / Elevator+Dumper icin):
 *   Hub'a yakin (< 0.5m)  -> DUMP modu: 4.5V (yavas dokme, fuel dagitmaz)
 *   Hub'a orta  (0.5-1.2m) -> TOSS modu: 5.0-7.5V (orta gucte atma)
 *   Hub'a uzak  (1.2-2.5m) -> LAUNCH modu: 7.5-11.0V (guclu firlatma)
 *   Hub'a cok uzak (>2.5m) -> MOTOR DURDUR (fuel bosuna harcanmaz!)
 *
 * 2026 REBUILT Hub Bilgileri:
 *   - Fuel: 5.91 inch (~15cm) cap, high-density foam top
 *   - Hub'a skor: Fuel, hub'in ust acikligindan girip sensor dizisinden gecmeli
 *   - Her alliance'in 1 Hub'i var (Red Hub, Blue Hub)
 *   - Big Dumper: Elevator ile yukselip hub'in ustune doker
 *   - Hub Zone (~2m yaricap) icinde olmak ideal
 * ============================================================================
 */
public class DistanceBasedShooterCommand extends Command {

    private final KrakenMotorSubsystem motor;
    private final VisionSubsystem vision; // nullable - olmayabilir
    private final String limelightName;

    // ========================================================================
    // MESAFE ESIKLERI - Big Dumper icin optimize edilmis
    // Hub'in ust acikligina dump/toss/launch icin farkli bolgeler
    // ========================================================================
    
    /** Hub'a en yakin mesafe - daha yakin olunca dump mantigi baslar */
    private static final double DUMP_RANGE_MAX = 0.5;   // metre
    
    /** Orta mesafe siniri - toss modu */
    private static final double TOSS_RANGE_MAX = 1.2;    // metre
    
    /** Maksimum etkili atim mesafesi - bunun otesinde motor calismaz */
    private static final double LAUNCH_RANGE_MAX = 2.5;  // metre
    
    // ========================================================================
    // VOLTAJ PROFILI - Her bolge icin min/max voltaj
    // Big Dumper elevator dump mekanizmasi icin kalibre edilmeli!
    // ========================================================================
    
    /** Dump modu voltaji (hub'a cok yakin, yavas dokme) */
    private static final double DUMP_VOLTAGE = 4.5;
    
    /** Toss modu voltaj araligi */
    private static final double TOSS_MIN_VOLTAGE = 5.0;
    private static final double TOSS_MAX_VOLTAGE = 7.5;
    
    /** Launch modu voltaj araligi */
    private static final double LAUNCH_MIN_VOLTAGE = 7.5;
    private static final double LAUNCH_MAX_VOLTAGE = 11.0;
    
    /** Odometry/vision yoksa fallback voltaj */
    private static final double DEFAULT_VOLTAGE = 6.0;

    // ========================================================================
    // STATE
    // ========================================================================
    private double hubDistance = -1.0;
    private double currentVoltage = 0.0;
    private String currentMode = "IDLE";
    private boolean usingOdometry = false;

    // ========================================================================
    // CONSTRUCTOR - VisionSubsystem ile (odometry + hub mesafesi)
    // ========================================================================
    
    /**
     * Tam ozellikli constructor - odometry bazli hub mesafesi kullanir.
     * VisionSubsystem uzerinden robotun sahadaki gercek konumunu alarak
     * kendi alliance'inin hub'ina olan mesafeyi hesaplar.
     * 
     * @param motor KrakenMotorSubsystem (dumper motoru)
     * @param vision VisionSubsystem (odometry + alliance hub mesafesi)
     * @param limelightName Limelight ismi (fallback icin)
     */
    public DistanceBasedShooterCommand(KrakenMotorSubsystem motor, VisionSubsystem vision, String limelightName) {
        this.motor = motor;
        this.vision = vision;
        this.limelightName = limelightName;
        addRequirements(motor);
    }
    
    /**
     * Basit constructor - sadece Limelight kamera mesafesi kullanir.
     * VisionSubsystem yoksa veya test icin kullanilir.
     * 
     * @param motor KrakenMotorSubsystem (dumper motoru)
     * @param limelightName Limelight ismi
     */
    public DistanceBasedShooterCommand(KrakenMotorSubsystem motor, String limelightName) {
        this(motor, null, limelightName);
    }

    @Override
    public void initialize() {
        hubDistance = -1.0;
        currentVoltage = 0.0;
        currentMode = "INITIALIZING";
        usingOdometry = false;
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    @Override
    public void execute() {
        // ==================================================================
        // ADIM 1: MESAFE HESAPLA (Oncelik: Odometry > Kamera)
        // ==================================================================
        
        hubDistance = getHubDistance();
        
        // ==================================================================
        // ADIM 2: VOLTAJ BELIRLE (Mesafe bazli profil)
        // ==================================================================
        
        if (hubDistance < 0) {
            // Mesafe bilinmiyor -> guvenli default
            currentVoltage = DEFAULT_VOLTAGE;
            currentMode = "DEFAULT (mesafe yok)";
        } else if (hubDistance > LAUNCH_RANGE_MAX) {
            // Hub'a cok uzak -> motor durdur, fuel bosuna harcanmasin!
            currentVoltage = 0.0;
            currentMode = "TOO FAR (" + String.format("%.1fm", hubDistance) + " > " + LAUNCH_RANGE_MAX + "m)";
        } else if (hubDistance <= DUMP_RANGE_MAX) {
            // DUMP MODU: Hub'a cok yakin, yavas dok
            currentVoltage = DUMP_VOLTAGE;
            currentMode = "DUMP";
        } else if (hubDistance <= TOSS_RANGE_MAX) {
            // TOSS MODU: Orta mesafe, voltaj interpolasyonu
            double ratio = (hubDistance - DUMP_RANGE_MAX) / (TOSS_RANGE_MAX - DUMP_RANGE_MAX);
            currentVoltage = TOSS_MIN_VOLTAGE + ratio * (TOSS_MAX_VOLTAGE - TOSS_MIN_VOLTAGE);
            currentMode = "TOSS";
        } else {
            // LAUNCH MODU: Uzak mesafe, guclu firlatma
            double ratio = (hubDistance - TOSS_RANGE_MAX) / (LAUNCH_RANGE_MAX - TOSS_RANGE_MAX);
            currentVoltage = LAUNCH_MIN_VOLTAGE + ratio * (LAUNCH_MAX_VOLTAGE - LAUNCH_MIN_VOLTAGE);
            currentMode = "LAUNCH";
        }
        
        // ==================================================================
        // ADIM 3: MOTOR KONTROL
        // ==================================================================
        
        if (currentVoltage > 0.1) {
            motor.setVoltage(currentVoltage);
        } else {
            motor.stop();
        }

        // ==================================================================
        // ADIM 4: DASHBOARD TELEMETRI
        // ==================================================================
        
        SmartDashboard.putNumber("Dumper/HubDistance", hubDistance > 0 ? Math.round(hubDistance * 100.0) / 100.0 : -1);
        SmartDashboard.putNumber("Dumper/Voltage", Math.round(currentVoltage * 10.0) / 10.0);
        SmartDashboard.putString("Dumper/Mode", currentMode);
        SmartDashboard.putBoolean("Dumper/UsingOdometry", usingOdometry);
        SmartDashboard.putBoolean("Dumper/InRange", hubDistance > 0 && hubDistance <= LAUNCH_RANGE_MAX);
    }
    
    // ========================================================================
    // HUB MESAFE HESAPLAMA - Birincil: Odometry, Ikincil: Kamera
    // ========================================================================
    
    /**
     * Hub'a olan mesafeyi hesaplar.
     * 
     * Oncelik 1: VisionSubsystem varsa -> drivetrain odometry'den
     *   robotun sahadaki gercek X,Y koordinatini alir ve kendi
     *   alliance'inin Hub merkezine olan mesafeyi hesaplar.
     *   Bu en guvenilir yontemdir cunku:
     *   - FMS'e baglaninca alliance otomatik belirlenir
     *   - Vision fusion aktifse AprilTag ile duzeltilmis konum
     *   - Vision kapaliysa bile swerve odometry'den konum
     *   - Robot sahaya konuldugu andan itibaren konum bilir
     * 
     * Oncelik 2: Limelight kamera -> AprilTag gorunuyorsa
     *   kameradan mesafe hesaplar (CameraSpace pose)
     * 
     * @return Hub'a mesafe (metre), bilinmiyorsa -1
     */
    private double getHubDistance() {
        // --- ONCELIK 1: Odometry bazli hub mesafesi ---
        if (vision != null) {
            double odometryDist = vision.getDistanceToOwnHub();
            if (odometryDist >= 0 && odometryDist < 20.0) { // saha icinde mantikli deger
                usingOdometry = true;
                return odometryDist;
            }
        }
        
        // --- ONCELIK 2: Limelight kamera mesafesi (fallback) ---
        usingOdometry = false;
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        
        if (hasTarget) {
            double[] cameraPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
            
            if (cameraPose != null && cameraPose.length >= 3 && cameraPose[2] > 0.05) {
                double cx = cameraPose[0]; // x offset (kameradan sola/saga)
                double cz = cameraPose[2]; // z derinlik (kameradan uzaklik)
                double dist = Math.sqrt(cx * cx + cz * cz);
                
                if (dist > 0.1 && dist < 8.0) {
                    return dist;
                }
            }
        }
        
        return -1.0; // mesafe bilinmiyor
    }

    @Override
    public void end(boolean interrupted) {
        motor.stop();
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        SmartDashboard.putNumber("Dumper/Voltage", 0);
        SmartDashboard.putString("Dumper/Mode", "STOPPED");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    // ========================================================================
    // PUBLIC GETTERS - Diger komutlardan okunabilir
    // ========================================================================
    
    /** Suanki hub mesafesi (metre) */
    public double getHubDistanceMeters() { return hubDistance; }
    
    /** Suanki voltaj */
    public double getCurrentVoltage() { return currentVoltage; }
    
    /** Suanki mod (DUMP/TOSS/LAUNCH/TOO FAR/DEFAULT) */
    public String getCurrentMode() { return currentMode; }
    
    /** Odometry mi kamera mi kullaniliyor */
    public boolean isUsingOdometry() { return usingOdometry; }
    
    /** Hub menzilinde mi? */
    public boolean isInRange() { return hubDistance > 0 && hubDistance <= LAUNCH_RANGE_MAX; }
}
