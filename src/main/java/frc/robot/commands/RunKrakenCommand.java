package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenMotorSubsystem;

/**
 * ============================================================================
 * RUN KRAKEN COMMAND
 * ============================================================================
 * Kraken motoru belirtilen hizda calistirir.
 * Butona basildiginda calisir, birakilinca durur.
 *
 * Kullanim:
 *   button1.whileTrue(new RunKrakenCommand(krakenMotor, 1.0));   // ileri
 *   button2.whileTrue(new RunKrakenCommand(krakenMotor, -1.0));  // geri
 * ============================================================================
 */
public class RunKrakenCommand extends Command {

    private final KrakenMotorSubsystem motor;
    private final double speed;

    /**
     * @param motor KrakenMotorSubsystem instance
     * @param speed Hedef hiz: -1.0 (tam geri) ile 1.0 (tam ileri)
     */
    public RunKrakenCommand(KrakenMotorSubsystem motor, double speed) {
        this.motor = motor;
        this.speed = speed;
        addRequirements(motor);
    }

    @Override
    public void initialize() {
        motor.setSpeed(speed);
    }

    @Override
    public void execute() {
        // Hiz initialize'da ayarlandi, surekli tutuluyor
        // (periodic zaten SmartDashboard'a yaziyor)
    }

    @Override
    public void end(boolean interrupted) {
        motor.stop();
    }

    @Override
    public boolean isFinished() {
        // whileTrue ile kullanilacak — butona basili oldugu surece calisir
        return false;
    }
}
