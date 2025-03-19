package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class CorallatorSubsystem extends SubsystemBase {
    private SparkMax m_wrist = new SparkMax(12, MotorType.kBrushless);
    private RelativeEncoder m_wristEncoder = m_wrist.getEncoder();
    private SparkFlex m_corallator = new SparkFlex(13, MotorType.kBrushless);
    private SparkLimitSwitch m_limitSwitch = m_corallator.getForwardLimitSwitch();
    private SparkLimitSwitch m_wristLimitSwitch = m_wrist.getForwardLimitSwitch();
    private final double maxWristAngle = 38;
    private final PIDController m_pidController = new PIDController(.04, 0, 0);
    private final double targetAngleUpDegrees = 36; // FIND THE RIGHT ANGLES!!!!!!! :3
    private final double targetAngleDownDegrees = -7.5;
    private double setPoint = 0;
    private double flingerSpeed = 0.30;
    private boolean enablePID = true;
    private double manualSpeed = 0;

    public CorallatorSubsystem() {
        m_wristEncoder.setPosition(0);
        m_wrist.configure(Configs.CorallatorConfig.m_coralAngleator, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_corallator.configure(Configs.CorallatorConfig.m_corallatorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void angleUp() {
        setPoint = targetAngleUpDegrees;
        enablePID = true;
    }

    public void angleDown() {
        setPoint = targetAngleDownDegrees;
        enablePID = true;
    }


    public void intakeCoral() {
        m_corallator.set(flingerSpeed);
    }

    public void outtakeCoral() {
        m_corallator.set(-flingerSpeed);
    }

    private boolean hasCoral() {
        return m_limitSwitch.isPressed();
    }

    public void stopCoral() {
        m_corallator.set(0);
    }

    public boolean isWristSwitchPressed(){
        return m_wristLimitSwitch.isPressed();
    }

    public void setManualWristSpeed(double speed){
        manualSpeed = speed;
        enablePID = false;
    }

    @Override
    public void periodic() {
        double wristAngle = m_wristEncoder.getPosition();
        double wristSpeed = enablePID?m_pidController.calculate(m_wristEncoder.getPosition(), setPoint):manualSpeed;
        m_wrist.set(wristSpeed);
        SmartDashboard.putNumber("wrist encoder value", wristAngle);
        SmartDashboard.putNumber("wrist speed", wristSpeed);
        SmartDashboard.putBoolean("wrist switch pressed", m_wristLimitSwitch.isPressed());
        if (m_wristLimitSwitch.isPressed()){
            m_wristEncoder.setPosition(maxWristAngle);
        }
    }
}


