package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * SHOOT COMMAND - WCP PrepareShotCommand BIREBIR MANTIK
 *
 * RT basili tutuldugunca:
 *   1) Odometry ile hub mesafesi olculur
 *   2) Mesafeye gore InterpolatingTreeMap ile RPM + Hood enterpolasyon
 *   3) Shooter hizlanir (VelocityVoltage PID)
 *   4) Shooter hedef RPM'e ulasinca Feeder (5000 RPM) + Hopper baslar
 *   5) Birakinca hepsi durur, hood default'a doner
 *
 * Intake arm'a DOKUNULMAZ.
 */
public class ShootCommand extends Command {

    // ========================================================================
    // WCP Shot tablosu - InterpolatingTreeMap birebir kopya
    // ========================================================================
    private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
        new InterpolatingTreeMap<>(
            (startValue, endValue, q) ->
                InverseInterpolator.forDouble()
                    .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
            (startValue, endValue, t) ->
                new Shot(
                    Interpolator.forDouble()
                        .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                    Interpolator.forDouble()
                        .interpolate(startValue.hoodPosition, endValue.hoodPosition, t)
                )
        );

    /** WCP resmi degerler - 3 nokta, arasi enterpolasyon */
    static {
        distanceToShotMap.put(Inches.of(52.0),  new Shot(2800, 0.19));
        distanceToShotMap.put(Inches.of(114.4), new Shot(3275, 0.40));
        distanceToShotMap.put(Inches.of(165.5), new Shot(3650, 0.48));
    }

    // ========================================================================
    // Subsystemler
    // ========================================================================
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final HopperSubsystem hopper;
    private final VisionSubsystem vision;
    private final String limelightName;

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
        // 1) MESAFE - odometry ile hub mesafesi
        Distance distanceToHub = getDistanceToHub();

        // 2) ENTERPOLASYON - WCP birebir
        Shot shot = distanceToShotMap.get(distanceToHub);

        // 3) SHOOTER tam hiz + HOOD mesafeye gore
        shooter.runFull();  // DutyCycleOut(1.0) = tam hiz
        hood.setPosition(shot.hoodPosition);

        // 4) Feeder + hopper aninda baslar (shooter full power)
        feeder.feed();
        hopper.run();

        // Dashboard
        SmartDashboard.putNumber("Shoot/Distance (inches)", distanceToHub.in(Inches));
        SmartDashboard.putNumber("Shoot/Distance (m)", distanceToHub.in(Meters));
        SmartDashboard.putNumber("Shoot/Target RPM", shot.shooterRPM);
        SmartDashboard.putNumber("Shoot/Hood Pos", shot.hoodPosition);
        SmartDashboard.putBoolean("Shoot/ShooterReady", shooter.isVelocityWithinTolerance());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stop();
        hopper.stop();
        hood.setDefault();
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);

        SmartDashboard.putBoolean("Shoot/ShooterReady", false);
    }

    @Override
    public boolean isFinished() { return false; }

    // ========================================================================
    // WCP getDistanceToHub birebir kopya
    // ========================================================================
    private Distance getDistanceToHub() {
        if (vision != null) {
            double dist = vision.getDistanceToOwnHub();
            if (dist >= 0 && dist < 20.0) {
                return Meters.of(dist);
            }
        }
        // Fallback: Limelight kameradan direkt mesafe
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        if (hasTarget) {
            double[] pose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
            if (pose != null && pose.length >= 3 && pose[2] > 0.05) {
                double d = Math.sqrt(pose[0] * pose[0] + pose[2] * pose[2]);
                return Meters.of(d);
            }
        }
        // Mesafe alinamazsa en yakin noktayi varsay (1.32m)
        return Inches.of(52.0);
    }

    // ========================================================================
    // WCP Shot class birebir kopya
    // ========================================================================
    public static class Shot {
        public final double shooterRPM;
        public final double hoodPosition;

        public Shot(double shooterRPM, double hoodPosition) {
            this.shooterRPM = shooterRPM;
            this.hoodPosition = hoodPosition;
        }
    }
}
