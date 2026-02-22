package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.AlignToFrontOfTag;
import frc.robot.commands.DistanceBasedShooterCommand;
import frc.robot.commands.RunKrakenCommand;
import frc.robot.commands.auto.AutoAlignToTagCommand;
import frc.robot.commands.auto.TrackAprilTag;
import frc.robot.commands.auto.VisionAutoSeedCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KrakenMotorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain, "limelight");
    private final KrakenMotorSubsystem dumperMotor = new KrakenMotorSubsystem("Kraken", 10, "rio");
    private final KrakenMotorSubsystem intakeMotor = new KrakenMotorSubsystem("Kraken", 11, "rio");

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("dumperForward",
            Commands.runEnd(() -> dumperMotor.setSpeed(1.0), () -> dumperMotor.stop(), dumperMotor).withTimeout(3.0));

        NamedCommands.registerCommand("dumperReverse",
            Commands.runEnd(() -> dumperMotor.setSpeed(-1.0), () -> dumperMotor.stop(), dumperMotor).withTimeout(3.0));

        NamedCommands.registerCommand("dumperStop",
            Commands.runOnce(() -> dumperMotor.stop(), dumperMotor));

        NamedCommands.registerCommand("distanceShot",
            new DistanceBasedShooterCommand(dumperMotor, vision, "limelight").withTimeout(3.0));

        NamedCommands.registerCommand("alignToTag",
            new AutoAlignToTagCommand(drivetrain, "limelight", MaxSpeed, MaxAngularRate, 1.0, 0.25, 0.6));

        NamedCommands.registerCommand("visionOn", Commands.runOnce(() -> vision.setEnabled(true)));
        NamedCommands.registerCommand("visionOff", Commands.runOnce(() -> vision.setEnabled(false)));
    }

    /*
     * LOGITECH F310 BUTON HARITASI (XInput modu):
     *   1 (A)  -> Mesafeye dayali atis (Limelight + voltage)
     *   2 (B)  -> Tekerlekleri yone cevir
     *   3 (X)  -> Dumper ileri
     *   4 (Y)  -> Dumper geri
     *   LB     -> Field-centric sifirla
     *   RB     -> AprilTag donus hizalama
     *   RT     -> Yaklasip hizalama (1m)
     *   LT     -> Surekli tag takibi
     *   POV    -> Robot-centric / Vision on-off
     */
    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

        // 1 (A) -> Mesafeye dayali atis (Odometry + Hub mesafesi kullanir)
        joystick.a().whileTrue(new DistanceBasedShooterCommand(dumperMotor, vision, "limelight"));

        // 2 (B) -> Tekerlekleri yone cevir
        joystick.b().whileTrue(drivetrain.applyRequest(
            () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // 3 (X) -> Dumper ileri
        joystick.x().whileTrue(new RunKrakenCommand(dumperMotor, 1.0));

        // 4 (Y) -> Dumper geri
        joystick.y().whileTrue(new RunKrakenCommand(dumperMotor, -1.0));

        // RB -> Sadece donus hizalama
        joystick.rightBumper().whileTrue(
            new AlignToAprilTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate));

        // RT -> Mesafe + donus hizalama (1m)
        joystick.rightTrigger(0.5).whileTrue(
            new AlignToFrontOfTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate)
                .withDesiredDistance(1.0)
                .withSpeedScales(0.5, 0.6));

        // LT -> Surekli tag takibi
        joystick.leftTrigger(0.5).whileTrue(
            new TrackAprilTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate, 1.0, 0.5, 0.6));

        // LB -> Field-centric sifirla
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // POV
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.povDown().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
        joystick.povRight().onTrue(Commands.runOnce(() -> vision.setEnabled(true)));
        joystick.povLeft().onTrue(Commands.runOnce(() -> vision.setEnabled(false)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * ============================================================================
     * AUTONOMOUS COMMAND - VISION SEED DESTEKLI
     * ============================================================================
     * Sira:
     *   1) VisionAutoSeedCommand: Limelight ile AprilTag'leri gorup robotun
     *      gercek saha pozisyonunu tespit eder (max 0.75 sn)
     *   2) PathPlanner Auto: Secilen otonom rutini calisir. PathPlanner'in
     *      resetPose cagrisi IGNORE edilir cunku vision zaten gercek pozisyonu
     *      seed etmistir.
     * 
     * NEDEN?
     *   Robot sahada NEREYE konulursa konsun, vision ile gercek konumunu
     *   bilir ve PathPlanner yolunu oradan dogru sekilde takip eder.
     *   Artik PathPlanner'daki baslangic noktasi ile robotun fiziksel
     *   konumu uyusmak ZORUNDA DEGIL.
     */
    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        if (selected == null) return Commands.print("Otonom secilmedi!");
        
        return Commands.sequence(
            // ADIM 1: Vision ile gercek konumu bul ve odometry'e yaz
            new VisionAutoSeedCommand(drivetrain, vision, "limelight"),
            
            // ADIM 2: PathPlanner otonomunu calistir
            // (resetPose cagrisi vision seed varsa ignore edilecek)
            selected.asProxy()
        );
    }

    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
    public VisionSubsystem getVision() { return vision; }
    public KrakenMotorSubsystem getDumperMotor() { return dumperMotor; }
}
