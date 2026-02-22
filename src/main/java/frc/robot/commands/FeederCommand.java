package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FeederSubsystem;

/**
 * ============================================================================
 * FEEDER COMMAND - WCP Big Dumper 2026 REBUILT
 * ============================================================================
 * Feeder mekanizmasini ileri veya geri calistirir.
 * (Toplari shooter'a tek tek besleyen son asamadaki motor)
 *
 * Nasil calisir:
 *   1) Buton basilir -> Feeder belirtilen yonde 0.25 hizda calisir
 *   2) Buton birakilir -> Feeder durur (Brake mode, top kaymaz)
 *
 * NOT: Normal atis sirasinda Feeder AYRI butonla kullanilmaz!
 * ShootCommand icinde shooter hiza ulasinca feeder otomatik baslar.
 * Bu command sadece manuel kontrol / test icin vardir.
 *
 * Kullanim (RobotContainer'da veya otonom'da):
 *   new FeederCommand(feeder, FeederCommand.Direction.FEED)
 * ============================================================================
 */
public class FeederCommand extends Command {

    /**
     * Feeder hareket yonu.
     */
    public enum Direction {
        /** Topu shooter'a dogru besler */
        FEED,
        /** Topu geri cikarir */
        REVERSE
    }

    // ========================================================================
    // ALANLAR
    // ========================================================================
    private final FeederSubsystem feeder;
    private final Direction direction;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================

    /**
     * Feeder komutunu olusturur.
     *
     * @param feeder FeederSubsystem instance
     * @param direction Hareket yonu (FEED veya REVERSE)
     */
    public FeederCommand(FeederSubsystem feeder, Direction direction) {
        this.feeder = feeder;
        this.direction = direction;

        addRequirements(feeder);
    }

    // ========================================================================
    // COMMAND LIFECYCLE
    // ========================================================================

    @Override
    public void initialize() {
        loopCount = 0;
        if (direction == Direction.FEED) {
            feeder.feed();
        } else {
            feeder.reverse();
        }
    }

    @Override
    public void execute() {
        loopCount++;

        if (direction == Direction.FEED) {
            feeder.feed();
        } else {
            feeder.reverse();
        }

        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putString("Feeder/Command", direction.name());
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        SmartDashboard.putString("Feeder/Command", "STOPPED");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
