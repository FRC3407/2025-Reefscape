package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class CorallatorSubsystem extends SubsystemBase {
    private SparkMax m_wrist = new SparkMax(12, MotorType.kBrushless);
    private RelativeEncoder m_wristEncoder = m_wrist.getEncoder();
    private SparkFlex m_flinger = new SparkFlex(13, MotorType.kBrushless);
    private DigitalInput coralSensor = new DigitalInput(1);
    private final PIDController m_pidController = new PIDController(.02, 0, 0);
    private final double targetAngleUpDegrees = 8;
    private final double targetAngleDownDegrees = -11;
    private double set_point;
    public double flingerSpeed = 0.2;
    private boolean takingInCoral = false;
    private Timer timer = new Timer();

    public CorallatorSubsystem() {
        // set_point = m_wristEncoder.getPosition();
        set_point = targetAngleDownDegrees;
        m_wrist.configure(Configs.CorellatorModule.wristConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        m_flinger.configure(Configs.CorellatorModule.flingerConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public void angleUp() {
        set_point = targetAngleUpDegrees;
    }

    public void angleDown() {
        set_point = targetAngleDownDegrees;
    }

    public void intakeCoral() {
        takingInCoral = true;
        m_flinger.set(flingerSpeed);
    }

    public void outtakeCoral() {
        takingInCoral = false;
        m_flinger.set(-flingerSpeed);
        timer.reset();
        timer.start();
    }

    public void stopCoral() {
        m_flinger.set(0);
    }

    @Override
    public void periodic() {
        double wristAngle = m_wristEncoder.getPosition();
        double wristSpeed = m_pidController.calculate(m_wristEncoder.getPosition(), set_point);
        SmartDashboard.putNumber("wrist encoder value", wristAngle);
        // SmartDashboard.putNumber("wrist speed", wristSpeed);
        m_wrist.set(wristSpeed);
        if (takingInCoral && coralSensor.get()) {
            stopCoral();
        } else if (!takingInCoral && timer.hasElapsed(2.0)) {
            stopCoral();
        }
    }
}
