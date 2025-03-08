package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CorallatorSubsystem extends SubsystemBase{
    private SparkMax m_wrist = new SparkMax(12, MotorType.kBrushless);
    private RelativeEncoder m_wristEncoder = m_wrist.getEncoder();
    private SparkFlex m_flinger = new SparkFlex(13, MotorType.kBrushless);
    private final PIDController m_pidController = new PIDController(.02, 0, 0);
    private final double targetAngleUpDegrees = 8;
    private final double targetAngleDownDegrees = -11;
    private double setPoint;
    public double flingerSpeed = 0.05;
    private boolean justDropped = false; 
    private double dropTime = 0.0;
     
    public CorallatorSubsystem(){
        // set_point = m_wristEncoder.getPosition();
        setPoint = targetAngleDownDegrees;
    }
    public void angleUp(){
        setPoint = targetAngleUpDegrees;
    }
    public void angleDown(){
        setPoint = targetAngleDownDegrees;
    }
    public void intakeCoral(){
        m_flinger.set(flingerSpeed);
    }
    public void outtakeCoral(){
        m_flinger.set(-flingerSpeed);
        dropTime = System.currentTimeMillis() * 1000;
    }

    // FILL THIS WITH USEFUL STUFF WHEN L'ÉLECTRICIENNNNNNNNNN INSTALLS THE LIGHT THING
    private boolean isTakingCoral() {
        if (true) return false;
        else return true;
    }
    public void stopCoral() {
        m_flinger.set(0);
    }

    @Override
    public void periodic(){
        double wristAngle = m_wristEncoder.getPosition();
        double wristSpeed = m_pidController.calculate(m_wristEncoder.getPosition(), setPoint);
        SmartDashboard.putNumber("wrist encoder value", wristAngle);
        SmartDashboard.putNumber("wrist speed", wristSpeed);
        //m_wrist.set(wristSpeed);
    }
}
