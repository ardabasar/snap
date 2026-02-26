package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * SHOOT COMMAND - 2026 REBUILT
 *
 * RT basili tutuldugunce:
 *   - Shooter (CAN 9,10,11) = full power DutyCycle 1.0
 *   - Feeder (CAN 15) = full power DutyCycle 1.0 (shooter ile ayni)
 *   - Hopper (CAN 14) = calisir
 *   - Hood (PWM 3,4) = mesafe bazli servo pozisyon
 *
 * Birakinca hepsi durur, hood default'a doner.
 * Intake arm'a DOKUNULMAZ.
 */
public class ShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final HopperSubsystem hopper;
    private final VisionSubsystem vision;
    private final String limelightName;

    // Hood servo mesafe tablosu - WCP resmi degerler
    // Servo pozisyonu mesafeye gore degisir
    private static final double[][] HOOD_TABLE = {
        // { mesafe (metre), servo pozisyon }
        { 1.32, 0.19 },   //  52 inch - WCP
        { 2.91, 0.40 },   // 114 inch - WCP
        { 4.20, 0.48 },   // 165 inch - WCP
    };

    public ShootCommand(ShooterSubsystem shooter, HoodSubsystem hood,
                        FeederSubsystem feeder, HopperSubsystem hopper,
                        VisionSubsystem vision, String limelightName) {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.hopper = hopper;
        this.vision = vision;
        this.limelightName = limelightName;

        addRequirements(shooter, hood, feeder, hopper);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    @Override
    public void execute() {
        // Shooter + Feeder: FULL POWER
        shooter.runFull();
        feeder.feed();

        // Hopper: calisir
        hopper.run();

        // Hood: mesafe bazli servo
        double distance = getHubDistance();
        if (distance > 0) {
            double hoodPos = interpolateHood(distance);
            hood.setPosition(hoodPos);
        }

        // Dashboard
        SmartDashboard.putBoolean("Shoot/Active", true);
        SmartDashboard.putNumber("Shoot/Distance", distance > 0 ? Math.round(distance * 100.0) / 100.0 : -1);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stop();
        hopper.stop();
        hood.setDefault();
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);

        SmartDashboard.putBoolean("Shoot/Active", false);
    }

    @Override
    public boolean isFinished() { return false; }

    /** Hub mesafesi - odometry oncelikli, yoksa kamera. */
    private double getHubDistance() {
        if (vision != null) {
            double dist = vision.getDistanceToOwnHub();
            if (dist >= 0 && dist < 20.0) return dist;
        }
        // Kamera fallback
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        if (hasTarget) {
            double[] pose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
            if (pose != null && pose.length >= 3 && pose[2] > 0.05) {
                return Math.sqrt(pose[0] * pose[0] + pose[2] * pose[2]);
            }
        }
        return -1.0;
    }

    /** Hood servo pozisyonunu mesafeye gore enterpolasyonla hesapla. */
    private double interpolateHood(double distance) {
        if (distance <= HOOD_TABLE[0][0]) return HOOD_TABLE[0][1];
        if (distance >= HOOD_TABLE[HOOD_TABLE.length - 1][0]) return HOOD_TABLE[HOOD_TABLE.length - 1][1];

        for (int i = 0; i < HOOD_TABLE.length - 1; i++) {
            if (distance >= HOOD_TABLE[i][0] && distance <= HOOD_TABLE[i + 1][0]) {
                double ratio = (distance - HOOD_TABLE[i][0]) / (HOOD_TABLE[i + 1][0] - HOOD_TABLE[i][0]);
                return HOOD_TABLE[i][1] + ratio * (HOOD_TABLE[i + 1][1] - HOOD_TABLE[i][1]);
            }
        }
        return 0.35;
    }
}
