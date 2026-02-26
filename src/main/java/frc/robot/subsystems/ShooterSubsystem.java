package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ============================================================================
 * SHOOTER SUBSYSTEM - 2026 REBUILT
 * ============================================================================
 * 3 adet TalonFX (Kraken X60) ile surulen shooter mekanizmasi.
 *
 * Donanim:
 *   - Motor 1 (CAN 9):  +1 yon (CounterClockwise_Positive)
 *   - Motor 2 (CAN 10): -1 yon (Clockwise_Positive)
 *   - Motor 3 (CAN 11): -1 yon (Clockwise_Positive)
 *   - Coast mode (shooter serbest donebilmeli)
 *
 * Kontrol:
 *   - VelocityVoltage PID ile hedef RPM'e ulasir
 *   - Mesafe bazli RPM: ShootCommand tarafindan set edilir
 *
 * Shooter calisirken intake kolu (IntakeArmSubsystem) yarim hizda
 * calisiyor olacak - bu ShootCommand tarafindan yonetilir.
 * ============================================================================
 */
public class ShooterSubsystem extends SubsystemBase {

    // ========================================================================
    // CAN ID'LER
    // ========================================================================
    public static final int MOTOR_1_CAN_ID = 9;   // +1 yon
    public static final int MOTOR_2_CAN_ID = 10;  // -1 yon
    public static final int MOTOR_3_CAN_ID = 11;  // -1 yon
    public static final String CAN_BUS     = "rio";

    // ========================================================================
    // PID SABITLERI (VelocityVoltage Slot 0) - WCP RESMI DEGERLER
    // ========================================================================
    private static final double kP = 0.5;
    private static final double kI = 2.0;
    private static final double kD = 0.0;
    // kV = 12V / Kraken X60 free speed (6000 RPM = 100 RPS)
    private static final double kV = 12.0 / 100.0;  // 0.12
    private static final double kS = 0.0;

    /** Stator akim limiti (Amper) - WCP: 120A stator, 70A supply */
    private static final double STATOR_CURRENT_LIMIT = 120.0;
    private static final double SUPPLY_CURRENT_LIMIT = 70.0;

    /** RPM toleransi - hedefe bu kadar yaklasinca "hazir" sayilir */
    private static final double RPM_TOLERANCE = 150.0;

    // ========================================================================
    // DONANIM
    // ========================================================================
    private final TalonFX motor1;  // CAN 9,  +1 yon
    private final TalonFX motor2;  // CAN 10, -1 yon
    private final TalonFX motor3;  // CAN 11, -1 yon

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // ========================================================================
    // TELEMETRI
    // ========================================================================
    private final StatusSignal<AngularVelocity> vel1Signal;
    private final StatusSignal<AngularVelocity> vel2Signal;
    private final StatusSignal<AngularVelocity> vel3Signal;
    private final StatusSignal<Current> current1Signal;

    // ========================================================================
    // STATE
    // ========================================================================
    private double targetRPM = 0.0;
    private boolean running = false;
    private static final int DASHBOARD_INTERVAL = 10;
    private int loopCount = 0;

    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    public ShooterSubsystem() {
        motor1 = new TalonFX(MOTOR_1_CAN_ID, CAN_BUS);
        motor2 = new TalonFX(MOTOR_2_CAN_ID, CAN_BUS);
        motor3 = new TalonFX(MOTOR_3_CAN_ID, CAN_BUS);

        configureMotors();

        vel1Signal    = motor1.getRotorVelocity();
        vel2Signal    = motor2.getRotorVelocity();
        vel3Signal    = motor3.getRotorVelocity();
        current1Signal = motor1.getStatorCurrent();

        stop();
        System.out.println("[Shooter] Initialized - CAN " + MOTOR_1_CAN_ID
            + "(+1), " + MOTOR_2_CAN_ID + "(-1), " + MOTOR_3_CAN_ID + "(-1)");
    }

    // ========================================================================
    // MOTOR KONFIGURASYONU
    // ========================================================================
    private void configureMotors() {
        // Motor 1 config: +1 yon (varsayilan CounterClockwise_Positive)
        TalonFXConfiguration config1 = new TalonFXConfiguration();
        config1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config1.Slot0.kP = kP;
        config1.Slot0.kI = kI;
        config1.Slot0.kD = kD;
        config1.Slot0.kV = kV;
        config1.Slot0.kS = kS;
        config1.CurrentLimits.StatorCurrentLimitEnable = true;
        config1.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        config1.CurrentLimits.SupplyCurrentLimitEnable = true;
        config1.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        config1.Voltage.PeakReverseVoltage = 0;  // WCP: ters voltaj yok
        motor1.getConfigurator().apply(config1);

        // Motor 2 & 3: ayni config, sadece yon farkli
        applyConfig(motor2, InvertedValue.Clockwise_Positive);
        applyConfig(motor3, InvertedValue.Clockwise_Positive);
    }

    /** Tekrari onlemek icin config uygulama metodu */
    private void applyConfig(TalonFX motor, InvertedValue direction) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = direction;
        cfg.Slot0.kP = kP;
        cfg.Slot0.kI = kI;
        cfg.Slot0.kD = kD;
        cfg.Slot0.kV = kV;
        cfg.Slot0.kS = kS;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        cfg.Voltage.PeakReverseVoltage = 0;
        motor.getConfigurator().apply(cfg);
    }

    // ========================================================================
    // KONTROL
    // ========================================================================

    /** Shooter'i hedef RPM'de calistirir (3 motor birlikte).
     *  WCP ile birebir ayni: velocityRequest.withVelocity(RPM.of(rpm)) */
    public void setTargetRPM(double rpm) {
        targetRPM = Math.abs(rpm);
        running = true;
        // WCP: RPM.of(rpm) Units API kullanarak RPM gonderiyor
        motor1.setControl(velocityRequest.withVelocity(RPM.of(targetRPM)));
        motor2.setControl(velocityRequest.withVelocity(RPM.of(targetRPM)));
        motor3.setControl(velocityRequest.withVelocity(RPM.of(targetRPM)));
    }

    /** Percent output ile calistirir (WCP setPercentOutput ile ayni) */
    public void setPercentOutput(double percent) {
        running = percent != 0;
        targetRPM = 0;
        motor1.setControl(voltageRequest.withOutput(percent * 12.0));
        motor2.setControl(voltageRequest.withOutput(percent * 12.0));
        motor3.setControl(voltageRequest.withOutput(percent * 12.0));
    }

    /** Shooter'i durdurur. WCP ile ayni: setPercentOutput(0). */
    public void stop() {
        targetRPM = 0.0;
        running = false;
        setPercentOutput(0.0);
    }

    // ========================================================================
    // GETTER'LAR
    // ========================================================================
    public double getMotor1RPM() { return Math.abs(vel1Signal.refresh().getValue().in(RPM)); }
    public double getMotor2RPM() { return Math.abs(vel2Signal.refresh().getValue().in(RPM)); }
    public double getMotor3RPM() { return Math.abs(vel3Signal.refresh().getValue().in(RPM)); }
    public double getAverageRPM() { return (getMotor1RPM() + getMotor2RPM() + getMotor3RPM()) / 3.0; }
    public double getTargetRPM() { return targetRPM; }
    public boolean isRunning() { return running; }

    /** Tum motorlar hedef RPM toleransi icinde mi? */
    public boolean atTargetSpeed() {
        if (!running || targetRPM < 100) return false;
        return Math.abs(getMotor1RPM() - targetRPM) < RPM_TOLERANCE
            && Math.abs(getMotor2RPM() - targetRPM) < RPM_TOLERANCE
            && Math.abs(getMotor3RPM() - targetRPM) < RPM_TOLERANCE;
    }

    // ========================================================================
    // PERIODIC
    // ========================================================================
    @Override
    public void periodic() {
        loopCount++;
        if (loopCount % DASHBOARD_INTERVAL != 0) return;

        SmartDashboard.putNumber("Shooter/TargetRPM", Math.round(targetRPM));
        SmartDashboard.putNumber("Shooter/AvgRPM", Math.round(getAverageRPM()));
        SmartDashboard.putNumber("Shooter/Motor1RPM", Math.round(getMotor1RPM()));
        SmartDashboard.putNumber("Shooter/Motor2RPM", Math.round(getMotor2RPM()));
        SmartDashboard.putNumber("Shooter/Motor3RPM", Math.round(getMotor3RPM()));
        SmartDashboard.putNumber("Shooter/Current", Math.round(current1Signal.refresh().getValueAsDouble() * 10.0) / 10.0);
        SmartDashboard.putBoolean("Shooter/Running", running);
        SmartDashboard.putBoolean("Shooter/AtSpeed", atTargetSpeed());
    }
}
