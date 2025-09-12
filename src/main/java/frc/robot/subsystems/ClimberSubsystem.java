// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax m_climber = new SparkMax(climberCanId, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_climber.getEncoder();
  
  private double targetPoint;
  private boolean moving;
  private int direction;
  private int stage;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    if (moving){
      double targetDifference = m_encoder.getPosition()-targetPoint;
      m_climber.set(Math.signum(targetDifference)*climberSpeed);
      if (Math.signum(m_encoder.getPosition()-targetPoint) != direction){
        moving = false;
      }
    }
    else{
      m_climber.set(0);
    }

    // This method will be called once per scheduler run
  }

  public void setPosition(double target) {
    targetPoint = target;
    moving = true;
    direction = (int)Math.signum(m_encoder.getPosition()-targetPoint);
  }

  public void advanceClimber(){
    stage++;
    
  }

  public void reverseClimber(){
    stage--;
  }

  public void setClimber(){
    
  }
}
