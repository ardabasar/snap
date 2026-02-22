package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * AprilTag'in onune hassas ve yumusak sekilde hizalar (mesafe + donus).
 *
 * [FIX] Mesafe hesabi duzeltildi:
 *   cameraPose[2] (sadece Z) → sqrt(x^2 + z^2) (zemin projeksiyonu)
 *   Egik kamera monte edildiginde gercek yatay mesafeyi verir.
 *
 * [FIX] SmartDashboard throttle eklendi — crash onleme
 */
public class AlignToFrontOfTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    private final double maxSpeed;
    private final double maxAngularRate;

    // Donus PID
    private static final double ROT_KP = 0.035;
    private static final double ROT_KI = 0.0;
    private static final double ROT_KD = 0.003;

    // Mesafe PID
    private static final double DIST_KP = 0.8;
    private static final double DIST_KI = 0.0;
    private static final double DIST_KD = 0.05;

    private double desiredDistanceMeters = 1.0;

    private static final double ROT_TOLERANCE_DEG = 2.0;
    private static final double DIST_TOLERANCE_M  = 0.08;

    private static final double ROT_DEADBAND_DEG = 0.8;
    private static final double DIST_DEADBAND_M  = 0.05;

    private static final double MIN_ROT_OUTPUT  = 0.05;
    private static final double MIN_DIST_OUTPUT = 0.06;

    private static final double ROT_INVERT = 1.0;

    private double xScale   = 0.35;
    private double rotScale = 0.5;

    private static final double MAX_X_ACCEL   = 1.0;
    private static final double MAX_ROT_ACCEL = 3.0;

    // [FIX] Throttle — her loop'ta putString/putNumber crash yapar
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    private final PIDController rotPid;
    private final PIDController distPid;
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SlewRateLimiter xLimiter   = new SlewRateLimiter(MAX_X_ACCEL);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(MAX_ROT_ACCEL);

    // Son bilinen mesafe (tag kaybolursa koru)
    private double lastKnownDistance = 1.0;

    public AlignToFrontOfTag(CommandSwerveDrivetrain drivetrain, String limelightName,
            double maxSpeed, double maxAngularRate) {
        this.drivetrain    = drivetrain;
        this.limelightName = limelightName;
        this.maxSpeed      = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        rotPid = new PIDController(ROT_KP, ROT_KI, ROT_KD);
        rotPid.setSetpoint(0);
        rotPid.setTolerance(ROT_TOLERANCE_DEG);
        rotPid.enableContinuousInput(-180, 180);

        distPid = new PIDController(DIST_KP, DIST_KI, DIST_KD);
        distPid.setSetpoint(desiredDistanceMeters);
        distPid.setTolerance(DIST_TOLERANCE_M);

        addRequirements(drivetrain);
    }

    public AlignToFrontOfTag withDesiredDistance(double meters) {
        this.desiredDistanceMeters = MathUtil.clamp(meters, 0.3, 5.0);
        distPid.setSetpoint(this.desiredDistanceMeters);
        return this;
    }

    public AlignToFrontOfTag withSpeedScales(double xScale, double rotScale) {
        this.xScale   = MathUtil.clamp(xScale,   0.0, 1.0);
        this.rotScale = MathUtil.clamp(rotScale, 0.0, 1.0);
        return this;
    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
        rotPid.reset();
        distPid.reset();
        xLimiter.reset(0);
        rotLimiter.reset(0);
        loopCount        = 0;
        lastKnownDistance = desiredDistanceMeters;

        SmartDashboard.putString("AlignFront/Status", "Aligning...");
        SmartDashboard.putNumber("AlignFront/TargetDist", desiredDistanceMeters);
    }

    @Override
    public void execute() {
        loopCount++;
        boolean shouldLog = (loopCount % DASHBOARD_INTERVAL == 0);

        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        if (!hasTarget) {
            drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            xLimiter.reset(0);
            rotLimiter.reset(0);
            if (shouldLog) SmartDashboard.putString("AlignFront/Status", "No Tag");
            return;
        }

        double tx = LimelightHelpers.getTX(limelightName);

        // ====================================================================
        // MESAFE OLCUMU
        // [FIX] cameraPose[2] (sadece Z ekseni) yerine sqrt(x^2+z^2) kullanilir.
        //
        // VisionSubsystem'de getTargetPose_CameraSpace'den gelen array formati:
        //   [0]=x (yatay), [1]=y (dikey), [2]=z (derinlik/uzaklik)
        //
        // Kamera egik monte edilmisse (tipik FRC: ~20-30 derece asagi bakan):
        //   - cameraPose[2] = optical axis mesafesi (Z ekseni)
        //   - Gercek zemin mesafesi: robotun merkezinden tag'e yatay mesafe
        //   - sqrt(x^2 + z^2): Y'yi (dikey) yoksayarak zemin projeksiyonu
        //   - Kamera 20° egik, 1.5m mesafede → Z=1.41m ama gercek=1.50m
        //     Duzeltilmis: sqrt(0.51^2 + 1.41^2) ≈ 1.50m (dogru!)
        // ====================================================================
        double measuredDistance;
        double[] cameraPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);

        if (cameraPose != null && cameraPose.length >= 3 && cameraPose[2] > 0.05) {
            double cx = cameraPose[0]; // yatay offset
            double cz = cameraPose[2]; // derinlik
            measuredDistance = MathUtil.clamp(
                Math.sqrt(cx * cx + cz * cz), 0.1, 6.0
            );
            lastKnownDistance = measuredDistance;
        } else {
            double ta = LimelightHelpers.getTA(limelightName);
            measuredDistance = estimateDistanceFromArea(ta);
        }

        // ====================================================================
        // DONUS KONTROLU
        // ====================================================================
        double rotCmd = 0;
        if (Math.abs(tx) > ROT_DEADBAND_DEG) {
            double rotPidOutput = rotPid.calculate(tx);
            rotCmd = rotPidOutput * maxAngularRate * rotScale * ROT_INVERT;
            if (Math.abs(rotCmd) < MIN_ROT_OUTPUT) {
                rotCmd = Math.copySign(MIN_ROT_OUTPUT, rotCmd);
            }
            rotCmd = MathUtil.clamp(rotCmd, -maxAngularRate * rotScale, maxAngularRate * rotScale);
        }
        rotCmd = rotLimiter.calculate(rotCmd);

        // ====================================================================
        // MESAFE KONTROLU
        // ====================================================================
        double xCmd = 0;
        double distError = measuredDistance - desiredDistanceMeters;
        if (Math.abs(distError) > DIST_DEADBAND_M) {
            double distPidOutput = distPid.calculate(measuredDistance);
            xCmd = -distPidOutput * maxSpeed * xScale;
            if (Math.abs(xCmd) < MIN_DIST_OUTPUT) {
                xCmd = Math.copySign(MIN_DIST_OUTPUT, xCmd);
            }
            xCmd = MathUtil.clamp(xCmd, -maxSpeed * xScale, maxSpeed * xScale);
        }
        xCmd = xLimiter.calculate(xCmd);

        drivetrain.setControl(robotCentric
            .withVelocityX(xCmd)
            .withVelocityY(0)
            .withRotationalRate(rotCmd));

        // [FIX] Throttled — sadece 5Hz
        if (shouldLog) {
            boolean rotLocked  = Math.abs(tx)        < ROT_TOLERANCE_DEG;
            boolean distLocked = Math.abs(distError) < DIST_TOLERANCE_M;

            SmartDashboard.putBoolean("AlignFront/HasTarget",  true);
            SmartDashboard.putNumber("AlignFront/TX",          tx);
            SmartDashboard.putNumber("AlignFront/MeasuredDist", measuredDistance);
            SmartDashboard.putNumber("AlignFront/DistError",   distError);
            SmartDashboard.putNumber("AlignFront/RotCmd",      rotCmd);
            SmartDashboard.putNumber("AlignFront/XCmd",        xCmd);

            if (rotLocked && distLocked) {
                SmartDashboard.putString("AlignFront/Status", "LOCKED");
            } else if (rotLocked) {
                SmartDashboard.putString("AlignFront/Status", "Dist Only");
            } else if (distLocked) {
                SmartDashboard.putString("AlignFront/Status", "Rot Only");
            } else {
                SmartDashboard.putString("AlignFront/Status", "Aligning");
            }
        }
    }

    /**
     * Tag alanindan (ta) mesafeyi tahmin eder — yedek yontem.
     */
    private double estimateDistanceFromArea(double ta) {
        if (ta < 0.1) return lastKnownDistance;
        return MathUtil.clamp(25.0 / Math.sqrt(ta), 0.3, 5.0);
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        xLimiter.reset(0);
        rotLimiter.reset(0);
        SmartDashboard.putString("AlignFront/Status",
            interrupted ? "Cancelled" : "Done");
        SmartDashboard.putBoolean("AlignFront/HasTarget", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}