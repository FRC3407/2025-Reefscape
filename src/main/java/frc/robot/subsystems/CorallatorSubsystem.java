package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CorallatorSubsystem extends SubsystemBase{
    private SparkMax m_wrist = new SparkMax(0, MotorType.kBrushless);
    private RelativeEncoder m_wristEncoder = m_wrist.getEncoder();
    private SparkFlex m_flinger = new SparkFlex(0, null);
    private final PIDController m_pidController = new PIDController(1, 0, 0);
    private final double targetAngleUpDegrees = 45;
    private final double targetAngleDownDegrees = -45;
    private double set_point;
    public double flingerSpeed = 0.5; 
     
    public CorallatorSubsystem(){
        
    }
    public void angleUp(){
        set_point = targetAngleUpDegrees;
    }
    public void angleDown(){
        set_point = targetAngleDownDegrees;
    }
    public void intakeCoral(){
        m_flinger.set(flingerSpeed);
    }
    public void outtakeCoral(){
        m_flinger.set(-flingerSpeed)
    }
    public boolean isSensor(){

    }
    @Override
    public void periodic(){
        m_wrist.set(m_pidController.calculate(m_wristEncoder.getPosition(), set_point));
    }
}
