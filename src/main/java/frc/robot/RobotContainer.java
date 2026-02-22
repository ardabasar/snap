package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.StructureCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.ShootCommand;
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
 *
 * NOT: Feeder ayri buton degildir!
 * RT'ye basinca Shooter + Hood + Feeder birlikte calisir.
 * Buton birakilinca hepsi durur, hood servolari sifira doner.
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
        // Atis: Shooter + Hood + Feeder (shooter hazir olunca feeder baslar)
        NamedCommands.registerCommand("shoot",
            new ShootCommand(shooter, hood, feeder, vision, "limelight").withTimeout(3.0));

        // Feeder: Toplari shooter'a besle
        NamedCommands.registerCommand("feed",
            new FeederCommand(feeder, FeederCommand.Direction.FEED).withTimeout(2.0));

        // Feeder ters: Top cikarma
        NamedCommands.registerCommand("feedReverse",
            new FeederCommand(feeder, FeederCommand.Direction.REVERSE).withTimeout(2.0));

        // Elevator yukari
        NamedCommands.registerCommand("elevatorUp",
            new ElevatorCommand(elevator, ElevatorCommand.Direction.UP).withTimeout(3.0));

        // Elevator asagi
        NamedCommands.registerCommand("elevatorDown",
            new ElevatorCommand(elevator, ElevatorCommand.Direction.DOWN).withTimeout(3.0));

        // Structure ileri (toplari besle)
        NamedCommands.registerCommand("structureForward",
            new StructureCommand(structure, StructureCommand.Direction.FORWARD).withTimeout(3.0));

        // Structure geri
        NamedCommands.registerCommand("structureReverse",
            new StructureCommand(structure, StructureCommand.Direction.REVERSE).withTimeout(3.0));

        // Hizalama
        NamedCommands.registerCommand("alignToTag",
            new AutoAlignToTagCommand(drivetrain, "limelight", MaxSpeed, MaxAngularRate, 1.0, 0.25, 0.6));

        // Vision kontrol
        NamedCommands.registerCommand("visionOn", Commands.runOnce(() -> vision.setEnabled(true)));
        NamedCommands.registerCommand("visionOff", Commands.runOnce(() -> vision.setEnabled(false)));
    }

    /*
     * ========================================================================
     * XBOX CONTROLLER BUTON HARITASI
     * ========================================================================
     *
     *  STICKS:
     *    Sol Stick       -> Swerve surme (field-centric X/Y)
     *    Sag Stick X     -> Donus (rotation)
     *
     *  FACE BUTONLARI:
     *    A (alt)         -> Elevator asagi (0.25 hiz, basili tut)
     *    B (sag)         -> Structure ileri - toplari besle (basili tut)
     *    X (sol)         -> Structure geri (basili tut)
     *    Y (ust)         -> Elevator yukari (0.25 hiz, basili tut)
     *
     *  BUMPER / TRIGGER:
     *    LB              -> Field-centric sifirla (heading reset)
     *    RB              -> AprilTag donus hizalama (basili tut)
     *    RT              -> ATIS! Shooter + Hood + Feeder birlikte
     *                       (Limelight mesafe -> RPM + aci, feeder otomatik)
     *    LT              -> Surekli AprilTag takibi (basili tut)
     *
     *  D-PAD:
     *    D-Pad Up        -> Vision ON
     *    D-Pad Down      -> Vision OFF
     *
     *  MENU:
     *    Start           -> (bos)
     *    Back            -> (bos)
     *
     * ========================================================================
     */
    private void configureBindings() {

        // ==================================================================
        // SWERVE SURME (varsayilan komut)
        // Sol stick X/Y -> field-centric hareket
        // Sag stick X   -> donus
        // ==================================================================
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

        // ==================================================================
        // RT (Sag Trigger) -> ATIS KOMUTU (ShootCommand)
        //   1) Limelight ile hub mesafesi olculur
        //   2) Mesafeye gore RPM + hood acisi enterpolasyonla belirlenir
        //   3) Shooter motorlari hedef RPM'e hizlanir
        //   4) Shooter HAZIR olunca -> Feeder otomatik baslar (0.25 hiz)
        //   5) Buton birakilinca hepsi durur, hood sifira doner
        // ==================================================================
        joystick.rightTrigger(0.5).whileTrue(
            new ShootCommand(shooter, hood, feeder, vision, "limelight"));

        // ==================================================================
        // Y (ust) -> Elevator yukari (0.25 hiz, basili tutulunca)
        // ==================================================================
        joystick.y().whileTrue(
            new ElevatorCommand(elevator, ElevatorCommand.Direction.UP));

        // ==================================================================
        // A (alt) -> Elevator asagi (0.25 hiz, basili tutulunca)
        // ==================================================================
        joystick.a().whileTrue(
            new ElevatorCommand(elevator, ElevatorCommand.Direction.DOWN));

        // ==================================================================
        // B (sag) -> Structure ileri (toplari shooter'a besle)
        // ==================================================================
        joystick.b().whileTrue(
            new StructureCommand(structure, StructureCommand.Direction.FORWARD));

        // ==================================================================
        // X (sol) -> Structure geri
        // ==================================================================
        joystick.x().whileTrue(
            new StructureCommand(structure, StructureCommand.Direction.REVERSE));

        // ==================================================================
        // RB -> AprilTag donus hizalama (basili tut)
        // ==================================================================
        joystick.rightBumper().whileTrue(
            new AlignToAprilTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate));

        // ==================================================================
        // LT -> Surekli AprilTag takibi (basili tut)
        // ==================================================================
        joystick.leftTrigger(0.5).whileTrue(
            new TrackAprilTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate, 1.0, 0.5, 0.6));

        // ==================================================================
        // LB -> Field-centric sifirla (heading reset)
        // ==================================================================
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // ==================================================================
        // D-PAD UP -> Vision ON
        // ==================================================================
        joystick.povUp().onTrue(Commands.runOnce(() -> vision.setEnabled(true)));

        // ==================================================================
        // D-PAD DOWN -> Vision OFF
        // ==================================================================
        joystick.povDown().onTrue(Commands.runOnce(() -> vision.setEnabled(false)));

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
