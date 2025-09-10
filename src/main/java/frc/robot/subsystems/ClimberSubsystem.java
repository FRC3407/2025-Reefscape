package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClimberSubsystem extends SubsystemBase {

    private SparkMax climberMotor = new SparkMax(12, MotorType.kBrushless);
    private RelativeEncoder climberEncoder = climberMotor.getEncoder(); 
    private final PIDController climberController = new PIDController(0.1, 0.0, 0.0); 
    private boolean enablePID = false;
    private double setPoint = 0.0;

  public ClimberSubsystem() {
    climberEncoder.setPosition(0);
  }

  public void setClimberGoal(double d) {
    setPoint = d;
  }

  public void setEnabled(boolean b) {
    enablePID = b;
  }

  @Override
  public void periodic() {
    double climberPosition = climberEncoder.getPosition();
	SmartDashboard.putNumber("Climber Position", climberPosition);
    double motorSpeed = enablePID?climberController.calculate(climberPosition, setPoint):0.0;
    climberMotor.set(motorSpeed);
  }
}
