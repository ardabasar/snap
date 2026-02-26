package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.auto.AutoAlignToTagCommand;
import frc.robot.commands.auto.VisionAutoSeedCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * ============================================================================
 * ROBOT CONTAINER - 2026 REBUILT
 * ============================================================================
 * Tum subsystem'lerin ve command binding'lerinin merkezi.
 *
 * SUBSYSTEM'LER:
 *   - CommandSwerveDrivetrain: 4 modul swerve (Caracal CANivore, CAN 1-8)
 *   - VisionSubsystem: Limelight MegaTag2 lokalizasyon
 *   - ShooterSubsystem: 3x TalonFX (CAN 9 +yon, CAN 10-11 -yon)
 *   - IntakeArmSubsystem: 1x TalonFX encoder pozisyon (CAN 12)
 *   - IntakeRollerSubsystem: 1x TalonFX roller (CAN 13)
 *   - HopperSubsystem: 1x TalonFX kayis (CAN 14)
 *   - FeederSubsystem: 1x TalonFX shooter beslemesi (CAN 15)
 *   - ClimbSubsystem: 1x TalonFX asma (CAN 16)
 *   - HoodSubsystem: 2x Servo pozisyon (PWM 3-4)
 *
 * CAN ID HARITASI:
 *   CANivore "Caracal" bus:
 *     Drive motorlari:  1, 3, 5, 7
 *     Steer motorlari:  2, 4, 6, 8
 *     CANcoders:        9, 10, 11, 12
 *     Pigeon2:          13
 *   rio bus:
 *     Shooter:          9 (+1 yon), 10 (-1 yon), 11 (-1 yon)
 *     Intake Arm:       12 (kol, encoder pozisyon)
 *     Intake Roller:    13 (silindir, 0.5 hiz)
 *     Hopper:           14 (kayis, -0.25 hiz)
 *     Feeder:           15 (shooter beslemesi, voltaj mantigi)
 *     Climb:            16 (+/-0.25 hiz)
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

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // ========================================================================
    // CONTROLLER
    // ========================================================================
    private final CommandXboxController joystick = new CommandXboxController(0);

    // ========================================================================
    // SUBSYSTEM'LER
    // ========================================================================
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision         = new VisionSubsystem(drivetrain, "limelight");
    private final ShooterSubsystem shooter       = new ShooterSubsystem();
    private final HoodSubsystem hood             = new HoodSubsystem();
    private final FeederSubsystem feeder         = new FeederSubsystem();
    private final HopperSubsystem hopper         = new HopperSubsystem();
    private final IntakeArmSubsystem intakeArm   = new IntakeArmSubsystem();
    private final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem();
    private final ClimbSubsystem climb           = new ClimbSubsystem();

    // ========================================================================
    // OTONOM
    // ========================================================================
    private final SendableChooser<Command> autoChooser;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public RobotContainer() {
        // Limelight kamera stream'ini dashboard'a gonder
        // Elastic'te "CameraServer" -> "limelight" olarak gorunur
        HttpCamera limelightCamera = new HttpCamera(
            "limelight",
            "http://10.95.45.11:5800/stream.mjpg",
            HttpCameraKind.kMJPGStreamer
        );
        CameraServer.startAutomaticCapture(limelightCamera);

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
        // Atis: Shooter + Hood + Feeder + Hopper + IntakeArm birlikte
        NamedCommands.registerCommand("shoot",
            new ShootCommand(shooter, hood, feeder, hopper, intakeArm, vision, "limelight")
                .withTimeout(3.0));

        // Intake: Arm + Roller
        NamedCommands.registerCommand("intake",
            new IntakeCommand(intakeArm, intakeRoller).withTimeout(3.0));

        // Hopper calistir
        NamedCommands.registerCommand("hopperRun",
            Commands.startEnd(() -> hopper.run(), () -> hopper.stop(), hopper).withTimeout(3.0));

        // Hizalama
        NamedCommands.registerCommand("alignToTag",
            new AutoAlignToTagCommand(drivetrain, "limelight", MaxSpeed, MaxAngularRate, 1.0, 0.25, 0.6));

        // Vision kontrol
        NamedCommands.registerCommand("visionOn", Commands.runOnce(() -> vision.setEnabled(true)));
        NamedCommands.registerCommand("visionOff", Commands.runOnce(() -> vision.setEnabled(false)));
    }

    /*
     * ========================================================================
     * XBOX CONTROLLER BUTON HARITASI - 2026 REBUILT
     * ========================================================================
     *
     *  STICKS (SADECE SURUS):
     *    Sol Stick       -> Swerve surme (field-centric X/Y)
     *    Sag Stick X     -> Donus (rotation)
     *
     *  FACE BUTONLARI:
     *    A (alt)         -> INTAKE (arm pozisyona git + roller)
     *    B (sag)         -> Hopper calistir (kayislari besle)
     *    X (sol)         -> Hopper ters (geriye al)
     *    Y (ust)         -> Climb yukari (+0.25)
     *
     *  BUMPER / TRIGGER:
     *    RB              -> AprilTag donus hizalama (basili tut)
     *    RT              -> ATIS! (Shooter+Feeder+Hood+Hopper+IntakeArm)
     *    LB              -> Intake kapat (arm stow pozisyonuna don)
     *    LT              -> Climb asagi (-0.25)
     *
     *  D-PAD:
     *    (bos - surus kontrolune karismaz)
     *
     *  MENU:
     *    Back            -> Field-centric sifirla (heading reset)
     *    Start           -> (bos)
     *
     *  OTOMATIK:
     *    Vision          -> HER ZAMAN ACIK
     *    Intake Arm      -> Otonomda tamamen acik, otonom bitince kapatilir
     *
     * ========================================================================
     */
    private void configureBindings() {

        // ==================================================================
        // SWERVE SURME (varsayilan komut - sadece stickler)
        // ==================================================================
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

        // ==================================================================
        // RT (Sag Trigger) -> ATIS
        //   Mesafe olc -> Shooter RPM + Hood aci ayarla
        //   Shooter hazir olunca -> Feeder + Hopper + IntakeArm (yarim hiz)
        //   Birakinca hepsi durur, hood sifira doner
        // ==================================================================
        joystick.rightTrigger(0.5).whileTrue(
            new ShootCommand(shooter, hood, feeder, hopper, intakeArm, vision, "limelight"));

        // ==================================================================
        // RB -> AprilTag donus hizalama (basili tut)
        // ==================================================================
        joystick.rightBumper().whileTrue(
            new AlignToAprilTag(drivetrain, "limelight", MaxSpeed, MaxAngularRate));

        // ==================================================================
        // A -> INTAKE (arm pozisyona git + roller baslar)
        // ==================================================================
        joystick.a().whileTrue(
            new IntakeCommand(intakeArm, intakeRoller));

        // ==================================================================
        // LB -> Intake kapat (arm stow / baslangic pozisyonuna don)
        // ==================================================================
        joystick.leftBumper().onTrue(
            Commands.runOnce(() -> intakeArm.goToPosition(0.0), intakeArm));

        // ==================================================================
        // B -> Hopper calistir (kayislari besle)
        // ==================================================================
        joystick.b().whileTrue(
            Commands.startEnd(() -> hopper.run(), () -> hopper.stop(), hopper));

        // ==================================================================
        // X -> Hopper ters (geriye al - top sikismasi vs.)
        // ==================================================================
        joystick.x().whileTrue(
            Commands.startEnd(() -> hopper.reverse(), () -> hopper.stop(), hopper));

        // ==================================================================
        // Y -> Climb yukari (+0.25)
        // ==================================================================
        joystick.y().whileTrue(
            new ClimbCommand(climb, ClimbCommand.Direction.UP));

        // ==================================================================
        // LT -> Climb asagi (-0.25)
        // ==================================================================
        joystick.leftTrigger(0.5).whileTrue(
            new ClimbCommand(climb, ClimbCommand.Direction.DOWN));

        // ==================================================================
        // Back -> Field-centric sifirla (heading reset)
        // ==================================================================
        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // ==================================================================
        // TELEMETRI
        // ==================================================================
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Autonomous command - vision seed destekli.
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
    public ShooterSubsystem getShooter() { return shooter; }
    public HoodSubsystem getHood() { return hood; }
    public FeederSubsystem getFeeder() { return feeder; }
    public HopperSubsystem getHopper() { return hopper; }
    public IntakeArmSubsystem getIntakeArm() { return intakeArm; }
    public IntakeRollerSubsystem getIntakeRoller() { return intakeRoller; }
    public ClimbSubsystem getClimb() { return climb; }
}
