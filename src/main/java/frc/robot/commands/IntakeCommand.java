package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * ============================================================================
 * INTAKE COMMAND - 2026 REBUILT
 * ============================================================================
 * Top alma komutu. Tusa basilinca:
 *   1) Intake arm (CAN 12) hedef pozisyona dogru hareket eder
 *   2) Arm hedef pozisyona ulasinca -> Intake roller (CAN 13) 0.5 hizda baslar
 *   3) Buton birakilinca arm durur, roller durur, arm pozisyonunda kalir (Brake)
 *
 * Arm Kraken dahili encoder ile pozisyon takip eder.
 * Baslangicta encoder 0, INTAKE_POSITION hedef deger.
 * ============================================================================
 */
public class IntakeCommand extends Command {

    private final IntakeArmSubsystem arm;
    private final IntakeRollerSubsystem roller;

    private enum IntakeState {
        /** Kol hedef pozisyona ilerliyor */
        ARM_MOVING,
        /** Kol pozisyonda, roller calisiyor */
        ROLLING
    }

    private IntakeState state = IntakeState.ARM_MOVING;

    // Dashboard throttle
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    public IntakeCommand(IntakeArmSubsystem arm, IntakeRollerSubsystem roller) {
        this.arm = arm;
        this.roller = roller;
        addRequirements(arm, roller);
    }

    @Override
    public void initialize() {
        state = IntakeState.ARM_MOVING;
        loopCount = 0;
    }

    @Override
    public void execute() {
        loopCount++;

        switch (state) {
            case ARM_MOVING:
                // Arm hedef pozisyona dogru hareket et
                double currentPos = arm.getPosition();
                double target = IntakeArmSubsystem.INTAKE_POSITION;

                if (currentPos < target) {
                    // Henuz hedef pozisyona ulasilmadi - ileri git
                    arm.setSpeed(IntakeArmSubsystem.ARM_SPEED);
                } else {
                    // Hedef pozisyona ulasti veya gecti - durdur ve roller baslat
                    arm.stop();
                    state = IntakeState.ROLLING;
                }

                // Arm hedefe yeterince yaklasmissa roller'i erken baslat
                if (arm.atPosition(target)) {
                    arm.stop();
                    state = IntakeState.ROLLING;
                }
                break;

            case ROLLING:
                // Arm pozisyonda, roller calisiyor
                arm.stop(); // Arm dursun (Brake tutar)
                roller.run(); // 0.5 hizda
                break;
        }

        // Dashboard
        if (loopCount % DASHBOARD_INTERVAL == 0) {
            SmartDashboard.putString("Intake/State", state.name());
            SmartDashboard.putNumber("Intake/ArmPos", Math.round(arm.getPosition() * 100.0) / 100.0);
            SmartDashboard.putBoolean("Intake/RollerActive", roller.isActive());
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
        roller.stop();
        SmartDashboard.putString("Intake/State", "STOPPED");
    }

    @Override
    public boolean isFinished() {
        // whileTrue ile kullanilir - buton basili oldugu surece
        return false;
    }
}
