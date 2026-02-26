package frc.robot.commands;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * SHOOT COMMAND - 2026 REBUILT
 * ============================================================================
 * Ana atis komutu. Limelight ile mesafe olcup, enterpolasyon ile
 * Shooter RPM ve Hood acisini ayarlar.
 *
 * CALISMA SIRASI:
 *   1) Limelight LED'leri ac, mesafe olc (odometry -> kamera fallback)
 *   2) Mesafeye gore RPM ve hood acisi enterpolasyonla hesapla
 *   3) Shooter (CAN 9,10,11) + Feeder (CAN 15) hedef RPM'e calistir
 *      - Feeder shooter ile AYNI ANDA voltaj mantigiyla calisir
 *   4) Hood servolari (PWM 3,4) hedef aciya git
 *   5) Hopper (CAN 14) -0.25 hizda kayislari calistir
 *   6) Intake arm (CAN 12) kolun degerinin YARISINDA calistir (top itme)
 *   7) Buton birakilinca: Hepsi durur, hood sifira doner
 *
 * NOT: Feeder artik shooter hazir olmasini BEKLEMIYOR.
 * Shooter + Feeder ayni voltaj mantigi ile birlikte calisir.
 * ============================================================================
 */
public class ShootCommand extends Command {

    // ========================================================================
    // SUBSYSTEM'LER
    // ========================================================================
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final HopperSubsystem hopper;
    private final IntakeArmSubsystem intakeArm;
    private final VisionSubsystem vision;
    private final String limelightName;

    // ========================================================================
    // ENTERPOLASYON TABLOLARI (mesafe metre -> deger)
    // ========================================================================
    private static final TreeMap<Double, Double> RPM_TABLE = new TreeMap<>();
    private static final TreeMap<Double, Double> HOOD_TABLE = new TreeMap<>();

    static {
        // WCP referans degerleri
        RPM_TABLE.put(1.32, 2800.0);   //  52 inch
        RPM_TABLE.put(2.06, 3000.0);   //  81 inch
        RPM_TABLE.put(2.90, 3275.0);   // 114 inch
        RPM_TABLE.put(3.48, 3425.0);   // 137 inch
        RPM_TABLE.put(3.81, 3500.0);   // 150 inch
        RPM_TABLE.put(4.19, 3650.0);   // 165 inch
        RPM_TABLE.put(5.08, 4100.0);   // 200 inch
        RPM_TABLE.put(5.72, 4500.0);   // 225 inch
        RPM_TABLE.put(6.35, 5000.0);   // 250 inch

        HOOD_TABLE.put(1.32, 0.19);
        HOOD_TABLE.put(2.06, 0.32);
        HOOD_TABLE.put(2.90, 0.40);
        HOOD_TABLE.put(3.48, 0.43);
        HOOD_TABLE.put(3.81, 0.46);
        HOOD_TABLE.put(4.19, 0.48);
        HOOD_TABLE.put(5.08, 0.55);
        HOOD_TABLE.put(5.72, 0.60);
        HOOD_TABLE.put(6.35, 0.65);
    }

    // ========================================================================
    // SABITLER
    // ========================================================================
    private static final double MIN_DISTANCE = 1.0;
    private static final double MAX_DISTANCE = 7.0;
    private static final double DEFAULT_RPM = 3200.0;
    private static final double DEFAULT_HOOD = 0.35;

    /** Feeder RPM orani - shooter RPM'inin bu kadari feeder'a verilir */
    private static final double FEEDER_RPM_RATIO = 0.6;

    /** Intake arm atis sirasindaki hiz orani (kolun degerinin yarisi) */
    private static final double ARM_SHOOT_SPEED_RATIO = 0.5;

    // ========================================================================
    // STATE
    // ========================================================================
    private enum ShootState { SPINNING_UP, FIRING, OUT_OF_RANGE }
    private ShootState state = ShootState.SPINNING_UP;
    private double currentDistance = -1.0;
    private double currentRPM = 0.0;
    private double currentHoodPos = 0.0;
    private String distanceSource = "NONE";
    private static final int DASHBOARD_INTERVAL = 5;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public ShootCommand(ShooterSubsystem shooter, HoodSubsystem hood,
                        FeederSubsystem feeder, HopperSubsystem hopper,
                        IntakeArmSubsystem intakeArm, VisionSubsystem vision,
                        String limelightName) {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intakeArm = intakeArm;
        this.vision = vision;
        this.limelightName = limelightName;

        addRequirements(shooter, hood, feeder, hopper, intakeArm);
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
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    @Override
    public void execute() {
        loopCount++;

        // 1) MESAFE
        currentDistance = getHubDistance();

        // 2) RPM + HOOD HESAPLA
        if (currentDistance < 0) {
            currentRPM = DEFAULT_RPM;
            currentHoodPos = DEFAULT_HOOD;
        } else if (currentDistance > MAX_DISTANCE) {
            currentRPM = 0.0;
            currentHoodPos = HoodSubsystem.DEFAULT_POSITION;
            state = ShootState.OUT_OF_RANGE;
        } else {
            currentRPM = interpolate(RPM_TABLE, currentDistance);
            currentHoodPos = interpolate(HOOD_TABLE, currentDistance);
        }

        // 3) MOTOR KONTROL
        if (currentRPM > 100) {
            // Shooter: 3 motor hedef RPM'e
            shooter.setTargetRPM(currentRPM);

            // Feeder: Shooter ile ayni voltaj mantigi (RPM oranli)
            feeder.setTargetRPM(currentRPM * FEEDER_RPM_RATIO);

            // Hood: Servo pozisyon
            hood.setPosition(currentHoodPos);

            // Hopper: Kayislari calistir (-0.25 hiz)
            hopper.run();

            // Intake arm: Kolun degerinin YARISI kadar calistir (top itme)
            intakeArm.setSpeed(IntakeArmSubsystem.ARM_SPEED * ARM_SHOOT_SPEED_RATIO);

            // State guncelle
            if (shooter.atTargetSpeed()) {
                state = ShootState.FIRING;
            } else {
                state = ShootState.SPINNING_UP;
            }
        } else {
            // Menzil disi - hepsini durdur
            shooter.stop();
            feeder.stop();
            hopper.stop();
            intakeArm.stop();
            hood.setDefault();
            state = ShootState.OUT_OF_RANGE;
        }

        // 4) DASHBOARD
        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putString("Shoot/State", state.name());
            SmartDashboard.putNumber("Shoot/Distance",
                currentDistance > 0 ? Math.round(currentDistance * 100.0) / 100.0 : -1);
            SmartDashboard.putNumber("Shoot/TargetRPM", Math.round(currentRPM));
            SmartDashboard.putNumber("Shoot/ActualRPM", Math.round(shooter.getAverageRPM()));
            SmartDashboard.putNumber("Shoot/HoodPos", Math.round(currentHoodPos * 1000.0) / 1000.0);
            SmartDashboard.putString("Shoot/Source", distanceSource);
            SmartDashboard.putBoolean("Shoot/ShooterReady", shooter.atTargetSpeed());
            SmartDashboard.putBoolean("Shoot/Firing", state == ShootState.FIRING);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stop();
        hopper.stop();
        intakeArm.stop();
        hood.setDefault();
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);

        SmartDashboard.putString("Shoot/State", "STOPPED");
        SmartDashboard.putNumber("Shoot/TargetRPM", 0);
        SmartDashboard.putBoolean("Shoot/ShooterReady", false);
        SmartDashboard.putBoolean("Shoot/Firing", false);
    }

    @Override
    public boolean isFinished() { return false; }

    // ========================================================================
    // MESAFE
    // ========================================================================
    private double getHubDistance() {
        if (vision != null) {
            double odometryDist = vision.getDistanceToOwnHub();
            if (odometryDist >= 0 && odometryDist < 20.0) {
                distanceSource = "ODOMETRY";
                return odometryDist;
            }
        }
        distanceSource = "CAMERA";
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        if (hasTarget) {
            double[] cameraPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
            if (cameraPose != null && cameraPose.length >= 3 && cameraPose[2] > 0.05) {
                double cx = cameraPose[0];
                double cz = cameraPose[2];
                double dist = Math.sqrt(cx * cx + cz * cz);
                if (dist > 0.1 && dist < 10.0) return dist;
            }
        }
        distanceSource = "NONE";
        return -1.0;
    }

    // ========================================================================
    // ENTERPOLASYON
    // ========================================================================
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
    // PUBLIC GETTER'LAR
    // ========================================================================
    public double getCurrentDistance() { return currentDistance; }
    public double getCurrentRPM() { return currentRPM; }
    public double getCurrentHoodPos() { return currentHoodPos; }
    public ShootState getState() { return state; }
    public boolean isFiring() { return state == ShootState.FIRING; }
}
