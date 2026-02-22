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
import frc.robot.commands.PrepareShotCommand;
import frc.robot.commands.auto.AutoAlignToTagCommand;
import frc.robot.commands.auto.TrackAprilTag;
import frc.robot.commands.auto.VisionAutoSeedCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StructureSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * ROBOT CONTAINER - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * Tum subsystem'lerin ve command binding'lerinin merkezi.
 *
 * SUBSYSTEM'LER:
 *   - CommandSwerveDrivetrain: 4 modul swerve (Caracal CANivore, CAN 1-8)
 *   - VisionSubsystem: Limelight MegaTag2 lokalizasyon
 *   - ElevatorSubsystem: 2x TalonFX leader-follower (CAN 9-10)
 *   - StructureSubsystem: 2x TalonFX leader-follower (CAN 11-12)
 *   - ShooterSubsystem: 3x TalonFX VelocityVoltage PID (CAN 13-14-15)
 *   - HoodSubsystem: 2x Servo pozisyon (PWM 3-4)
 *   - FeederSubsystem: 1x TalonFX DutyCycle (CAN 16)
 *
 * CAN ID HARITASI:
 *   CANivore "Caracal" bus:
 *     Drive motorlari:  1, 3, 5, 7
 *     Steer motorlari:  2, 4, 6, 8
 *     CANcoders:        9, 10, 11, 12
 *     Pigeon2:          13
 *   rio bus:
 *     Elevator:         9 (leader), 10 (follower)
 *     Structure:        11 (leader), 12 (follower)
 *     Shooter:          13 (sol), 14 (orta), 15 (sag)
 *     Feeder:           16
 *   PWM:
 *     Hood servolar:    3 (sol), 4 (sag)
 * ============================================================================
 */
public class RobotContainer {

    // ========================================================================
    // SWERVE SABITLERI
    // ========================================================================
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond);

    // ========================================================================
    // SWERVE REQUEST'LER
    // ========================================================================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // ========================================================================
    // CONTROLLER
    // ========================================================================
    private final CommandXboxController joystick = new CommandXboxController(0);

    // ========================================================================
    // SUBSYSTEM'LER
    // ========================================================================
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision       = new VisionSubsystem(drivetrain, "limelight");
    private final ElevatorSubsystem elevator   = new ElevatorSubsystem();
    private final StructureSubsystem structure = new StructureSubsystem();
    private final ShooterSubsystem shooter     = new ShooterSubsystem();
    private final HoodSubsystem hood           = new HoodSubsystem();
    private final FeederSubsystem feeder       = new FeederSubsystem();

    // ========================================================================
    // OTONOM
    // ========================================================================
    private final SendableChooser<Command> autoChooser;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public RobotContainer() {
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    // ========================================================================
    // NAMED COMMANDS (PathPlanner Otonom icin)
    // ========================================================================
    private void registerNamedCommands() {
        // Shooter: Mesafe bazli atisa hazirla (shooter + hood birlikte)
        NamedCommands.registerCommand("prepareShot",
            new PrepareShotCommand(shooter, hood, vision, "limelight").withTimeout(3.0));

        // Feeder: Toplari shooter'a besle
        NamedCommands.registerCommand("feed",
            Commands.runEnd(() -> feeder.feed(), () -> feeder.stop(), feeder).withTimeout(2.0));

        // Feeder ters: Top cikarma
        NamedCommands.registerCommand("feedReverse",
            Commands.runEnd(() -> feeder.reverse(), () -> feeder.stop(), feeder).withTimeout(2.0));

        // Elevator yukari
        NamedCommands.registerCommand("elevatorUp",
            Commands.runEnd(() -> elevator.moveUp(), () -> elevator.stop(), elevator).withTimeout(3.0));

        // Elevator asagi
        NamedCommands.registerCommand("elevatorDown",
            Commands.runEnd(() -> elevator.moveDown(), () -> elevator.stop(), elevator).withTimeout(3.0));

        // Structure ileri (toplari besle)
        NamedCommands.registerCommand("structureForward",
            Commands.runEnd(() -> structure.feedForward(), () -> structure.stop(), structure).withTimeout(3.0));

        // Structure geri
        NamedCommands.registerCommand("structureReverse",
            Commands.runEnd(() -> structure.feedReverse(), () -> structure.stop(), structure).withTimeout(3.0));

        // Hizalama
        NamedCommands.registerCommand("alignToTag",
            new AutoAlignToTagCommand(drivetrain, "limelight", MaxSpeed, MaxAngularRate, 1.0, 0.25, 0.6));

        // Vision kontrol
        NamedCommands.registerCommand("visionOn", Commands.runOnce(() -> vision.setEnabled(true)));
        NamedCommands.registerCommand("visionOff", Commands.runOnce(() -> vision.setEnabled(false)));
    }

    /*
     * ========================================================================
     * LOGITECH F310 / XBOX CONTROLLER BUTON HARITASI (XInput modu)
     * ========================================================================
     *
     *   A buton  -> Mesafe bazli atis (Shooter + Hood: Limelight mesafe -> RPM + aci)
     *   B buton  -> Tekerlekleri yone cevir (swerve debug)
     *   X buton  -> Feeder ileri (toplari shooter'a besle)
     *   Y buton  -> Feeder geri (top cikarma)
     *
     *   LB       -> Field-centric sifirla (heading reset)
     *   RB       -> AprilTag donus hizalama
     *   RT       -> AprilTag yaklasip hizalama (1m mesafe)
     *   LT       -> Surekli AprilTag takibi
     *
     *   Sol Stick -> Swerve surme (field-centric X/Y)
     *   Sag Stick X -> Donus (rotation)
     *
     *   POV Up    -> Elevator yukari (0.25 hiz)
     *   POV Down  -> Elevator asagi (0.25 hiz)
     *   POV Right -> Structure ileri (toplari besle)
     *   POV Left  -> Structure geri
     *
     *   Start buton -> Vision ON
     *   Back buton  -> Vision OFF
     *
     * ========================================================================
     */
    private void configureBindings() {

        // ==================================================================
        // SWERVE SURME (varsayilan komut)
        // ==================================================================
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

        // ==================================================================
        // A BUTON -> Mesafe bazli atis (Shooter + Hood birlikte)
        // Limelight ile mesafe olcup RPM + hood acisi interpolasyonla belirlenir
        // ==================================================================
        joystick.a().whileTrue(
            new PrepareShotCommand(shooter, hood, vision, "limelight"));

        // ==================================================================
        // B BUTON -> Tekerlekleri yone cevir (swerve debug)
        // ==================================================================
        joystick.b().whileTrue(drivetrain.applyRequest(
            () -> point.withModuleDirection(
                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // ==================================================================
        // X BUTON -> Feeder ileri (toplari shooter'a besle, 0.25 hiz)
        // ==================================================================
        joystick.x().whileTrue(
            Commands.runEnd(() -> feeder.feed(), () -> feeder.stop(), feeder));

        // ==================================================================
        // Y BUTON -> Feeder geri (top cikarma)
        // ==================================================================
        joystick.y().whileTrue(
            Commands.runEnd(() -> feeder.reverse(), () -> feeder.stop(), feeder));

        // ==================================================================
        // RB -> AprilTag donus hizalama
        // ==================================================================
        joystick.rightBumper().whileTrue(
            new AlignToAprilTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate));

        // ==================================================================
        // RT -> Mesafe + donus hizalama (1m hedef)
        // ==================================================================
        joystick.rightTrigger(0.5).whileTrue(
            new AlignToFrontOfTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate)
                .withDesiredDistance(1.0)
                .withSpeedScales(0.5, 0.6));

        // ==================================================================
        // LT -> Surekli AprilTag takibi
        // ==================================================================
        joystick.leftTrigger(0.5).whileTrue(
            new TrackAprilTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate, 1.0, 0.5, 0.6));

        // ==================================================================
        // LB -> Field-centric sifirla (heading reset)
        // ==================================================================
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // ==================================================================
        // POV UP -> Elevator yukari (0.25 hiz, basili tutulunca)
        // ==================================================================
        joystick.povUp().whileTrue(
            Commands.runEnd(() -> elevator.moveUp(), () -> elevator.stop(), elevator));

        // ==================================================================
        // POV DOWN -> Elevator asagi (0.25 hiz, basili tutulunca)
        // ==================================================================
        joystick.povDown().whileTrue(
            Commands.runEnd(() -> elevator.moveDown(), () -> elevator.stop(), elevator));

        // ==================================================================
        // POV RIGHT -> Structure ileri (toplari yukari besle)
        // ==================================================================
        joystick.povRight().whileTrue(
            Commands.runEnd(() -> structure.feedForward(), () -> structure.stop(), structure));

        // ==================================================================
        // POV LEFT -> Structure geri
        // ==================================================================
        joystick.povLeft().whileTrue(
            Commands.runEnd(() -> structure.feedReverse(), () -> structure.stop(), structure));

        // ==================================================================
        // START -> Vision ON / BACK -> Vision OFF
        // ==================================================================
        joystick.start().onTrue(Commands.runOnce(() -> vision.setEnabled(true)));
        joystick.back().onTrue(Commands.runOnce(() -> vision.setEnabled(false)));

        // ==================================================================
        // TELEMETRI
        // ==================================================================
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
     */
    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        if (selected == null) return Commands.print("Otonom secilmedi!");

        return Commands.sequence(
            new VisionAutoSeedCommand(drivetrain, vision, "limelight"),
            selected.asProxy()
        );
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================
    public CommandSwerveDrivetrain getDrivetrain() { return drivetrain; }
    public VisionSubsystem getVision() { return vision; }
    public ElevatorSubsystem getElevator() { return elevator; }
    public StructureSubsystem getStructure() { return structure; }
    public ShooterSubsystem getShooter() { return shooter; }
    public HoodSubsystem getHood() { return hood; }
    public FeederSubsystem getFeeder() { return feeder; }
}
