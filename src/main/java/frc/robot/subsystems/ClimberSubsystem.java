// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;

public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax m_climber = new SparkMax(climberCanId, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_climber.getEncoder();
  
  private double targetPoint;
  private boolean moving;
  private int direction;
  private int stage;
  private final Timer climbTimer = new Timer();
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_encoder.setPosition(0);
    m_climber.configure(Configs.ClimberConfig.m_climberConfig,ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
    climbTimer.reset(); 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber rotation:", m_encoder.getPosition());
    SmartDashboard.putNumber("Climber stage:", stage);
    SmartDashboard.putBoolean("Climber moving:", moving);
    if (climbTimer.hasElapsed(5)){
      if (moving){
        System.out.println("The climber took too long");
      }
      moving = false;
    }
    if (moving){
      double targetDifference = m_encoder.getPosition()-targetPoint;
      m_climber.set(Math.signum(targetDifference)*climberSpeed);
      if (Math.signum(m_encoder.getPosition()-targetPoint) != direction){
        moving = false;
        System.out.println("The climber made it to the next point");
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
    climbTimer.reset(); 
    climbTimer.start();
  }

  public void advanceClimber(int count){
    stage+=count;
    // if (stage < 0) stage = 2;
    // if (stage > 2) stage = 0; //no need to loop over
    setClimber();
  }

  public void climberForward(){
    advanceClimber(1);
  }

  public void climberBackward(){
    // advanceClimber(-1); //DONT go backwards!!!!!
  }

  public void setClimber(){
    if (stage == 0){
      setPosition(climberRestPosition);
    }
    if (stage == 1){
      setPosition(climberHookPosition);
    }
    if (stage == 2){
      setPosition(climberClimbedPosition);
    }
  }
}
