package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.StructureSubsystem;

/**
 * ============================================================================
 * STRUCTURE COMMAND - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * Structure mekanizmasini ileri veya geri hareket ettirir.
 * (Toplari alt kisimdan shooter'a besleyen mekanizma)
 *
 * Nasil calisir:
 *   1) Buton basilir -> Structure belirtilen yonde 0.25 hizda calisir
 *   2) Buton birakilir -> Structure durur (Brake mode)
 *   3) Encoder pozisyonu ve akim dashboard'a yazilir
 *
 * Kullanim (RobotContainer'da):
 *   joystick.b().whileTrue(new StructureCommand(structure, StructureCommand.Direction.FORWARD));
 *   joystick.x().whileTrue(new StructureCommand(structure, StructureCommand.Direction.REVERSE));
 * ============================================================================
 */
public class StructureCommand extends Command {

    /**
     * Structure hareket yonu.
     */
    public enum Direction {
        /** Toplari shooter'a dogru besler */
        FORWARD,
        /** Ters yone doner */
        REVERSE
    }

    // ========================================================================
    // ALANLAR
    // ========================================================================
    private final StructureSubsystem structure;
    private final Direction direction;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Structure komutunu olusturur.
     *
     * @param structure StructureSubsystem instance
     * @param direction Hareket yonu (FORWARD veya REVERSE)
     */
    public StructureCommand(StructureSubsystem structure, Direction direction) {
        this.structure = structure;
        this.direction = direction;

        addRequirements(structure);
    }

    // ========================================================================
    // COMMAND LIFECYCLE
    // ========================================================================

    @Override
    public void initialize() {
        loopCount = 0;
        if (direction == Direction.FORWARD) {
            structure.feedForward();
        } else {
            structure.feedReverse();
        }
    }

    @Override
    public void execute() {
        loopCount++;

        if (direction == Direction.FORWARD) {
            structure.feedForward();
        } else {
            structure.feedReverse();
        }

        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putString("Structure/Command", direction.name());
            SmartDashboard.putNumber("Structure/CmdPosition",
                Math.round(structure.getPositionRotations() * 100.0) / 100.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        structure.stop();
        SmartDashboard.putString("Structure/Command", "STOPPED");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
