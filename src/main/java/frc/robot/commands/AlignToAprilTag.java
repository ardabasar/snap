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
 * AprilTag'i ortaya hizalar (sadece dönüş).
 * ✅ SMOOTH & FAST ALIGN - Yumuşak ama hızlı hizalama!
 */
public class AlignToAprilTag extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final String limelightName;
  private final double maxAngularRate;

  // ============================================================
  // DENGELI VE YUMUSAK AYARLAR (orijinal tune edilmis degerler)
  // Tag kaybolunca yumusak durma eklendi (SlewRateLimiter 0'a ceker)
  // ============================================================

  // PID degerleri - Dengeli
  private static final double kP = 0.045;
  private static final double kI = 0.0;
  private static final double kD = 0.005;

  // Minimum donus hizi
  private static final double MIN_OUTPUT = 0.08;

  // Tolerans - Hedefe ulasildigi kabul araligi
  private static final double TOLERANCE_DEG = 1.5;

  // Deadband - Bu araliktaki hatay yoksay (titreme onler)
  private static final double DEADBAND_DEG = 0.5;

  // Robot TERS donuyorsa -1.0 yap
  private static final double INVERT = 1.0;

  // Donus hiz olcegi
  private double rotScale = 0.7;

  // Rate limiter - Yumusak ivmelenme
  private static final double MAX_ACCEL_RAD_PER_SEC_SQ = 5.0;

  // ============================================================

  private final PIDController pid;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(MAX_ACCEL_RAD_PER_SEC_SQ);

  public AlignToAprilTag(CommandSwerveDrivetrain drivetrain, String limelightName,
      double maxSpeed, double maxAngularRate) {
    this.drivetrain = drivetrain;
    this.limelightName = limelightName;
    this.maxAngularRate = maxAngularRate;

    // PID oluştur
    pid = new PIDController(kP, kI, kD);
    pid.setSetpoint(0); // Hedef: tx = 0 (ortada)
    pid.setTolerance(TOLERANCE_DEG);
    pid.enableContinuousInput(-180, 180);

    addRequirements(drivetrain);
  }

  public AlignToAprilTag withRotScale(double s) {
    this.rotScale = MathUtil.clamp(s, 0.0, 1.0);
    return this;
  }

  @Override
  public void initialize() {
    LimelightHelpers.setLEDMode_ForceOn(limelightName);
    pid.reset();
    rateLimiter.reset(0);
    SmartDashboard.putString("Align/Status", "🎯 Aligning...");
  }

  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(limelightName);
    SmartDashboard.putBoolean("Align/HasTarget", hasTarget);

    if (!hasTarget) {
      // Tag kayboldu -> YUMUSAK durma (SlewRateLimiter sifira ceker)
      // Sert durma yerine yumusak gecis -> tekerlek titremesi onlenir
      double softStop = rateLimiter.calculate(0);
      drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(softStop));
      if (Math.abs(softStop) < 0.01) {
        rateLimiter.reset(0);
      }
      SmartDashboard.putString("Align/Status", "No Tag");
      return;
    }

    double tx = LimelightHelpers.getTX(limelightName);
    SmartDashboard.putNumber("Align/TX", tx);

    // Deadband kontrolü
    if (Math.abs(tx) < DEADBAND_DEG) {
      drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      SmartDashboard.putString("Align/Status", "✅ Locked!");
      rateLimiter.reset(0);
      return;
    }

    // PID hesapla
    double pidOutput = pid.calculate(tx);
    SmartDashboard.putNumber("Align/PID_Output", pidOutput);

    // PID çıkışını dönüş hızına çevir
    double rotCmd = pidOutput * maxAngularRate * rotScale * INVERT;

    // Minimum çıkış uygula
    if (Math.abs(rotCmd) < MIN_OUTPUT) {
      rotCmd = Math.copySign(MIN_OUTPUT, rotCmd);
    }

    // Maksimum limit
    double maxRot = maxAngularRate * rotScale;
    rotCmd = MathUtil.clamp(rotCmd, -maxRot, maxRot);

    // Rate limiter - Hız değişimini yumuşat
    rotCmd = rateLimiter.calculate(rotCmd);

    SmartDashboard.putNumber("Align/RotCmd", rotCmd);
    SmartDashboard.putString("Align/Status", "⚡ " + String.format("%.2f", rotCmd));

    drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(rotCmd));
  }

  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setLEDMode_PipelineControl(limelightName);
    // Frenleme komutu kaldırıldı. Kontrol pürüzsüzce DefaultCommand'e geçecek.
    rateLimiter.reset(0);
    SmartDashboard.putString("Align/Status", interrupted ? "Cancelled" : "Done");
  }

  @Override
  public boolean isFinished() {
    // Bu komut basılı tutulduğu sürece çalışır, bu yüzden false kalmalı
    return false;
  }
}
