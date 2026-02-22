package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;

/**
 * ============================================================================
 * ELEVATOR COMMAND - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * Elevator'u joystick ile yukari veya asagi hareket ettirir.
 *
 * Nasil calisir:
 *   1) Buton basilir -> Elevator belirtilen yonde 0.25 hizda hareket eder
 *   2) Buton birakilir -> Elevator durur (Brake mode tutar pozisyonda)
 *   3) Encoder pozisyonu ve akim dashboard'a yazilir
 *
 * Kullanim (RobotContainer'da):
 *   joystick.y().whileTrue(new ElevatorCommand(elevator, ElevatorCommand.Direction.UP));
 *   joystick.a().whileTrue(new ElevatorCommand(elevator, ElevatorCommand.Direction.DOWN));
 * ============================================================================
 */
public class ElevatorCommand extends Command {

    /**
     * Elevator hareket yonu.
     */
    public enum Direction {
        UP,
        DOWN
    }

    // ========================================================================
    // ALANLAR
    // ========================================================================
    private final ElevatorSubsystem elevator;
    private final Direction direction;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Elevator komutunu olusturur.
     *
     * @param elevator ElevatorSubsystem instance
     * @param direction Hareket yonu (UP veya DOWN)
     */
    public ElevatorCommand(ElevatorSubsystem elevator, Direction direction) {
        this.elevator = elevator;
        this.direction = direction;

        // Bu command ElevatorSubsystem'i kullaniyor
        addRequirements(elevator);
    }

    // ========================================================================
    // COMMAND LIFECYCLE
    // ========================================================================

    @Override
    public void initialize() {
        loopCount = 0;
        // Motorlari baslat
        if (direction == Direction.UP) {
            elevator.moveUp();
        } else {
            elevator.moveDown();
        }
    }

    @Override
    public void execute() {
        loopCount++;

        // Motorlari her loop'ta set et (guvenlik icin)
        if (direction == Direction.UP) {
            elevator.moveUp();
        } else {
            elevator.moveDown();
        }

        // Dashboard telemetri
        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putString("Elevator/Command", direction.name());
            SmartDashboard.putNumber("Elevator/CmdPosition",
                Math.round(elevator.getPositionRotations() * 100.0) / 100.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Buton birakilinca veya command kesildiyse -> DURDUR
        elevator.stop();

        SmartDashboard.putString("Elevator/Command", "STOPPED");
    }

    @Override
    public boolean isFinished() {
        // whileTrue ile kullanilir - buton basili oldugu surece calisir
        return false;
    }
}
