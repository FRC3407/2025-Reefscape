// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax m_climber = new SparkMax(climberCanId, MotorType.kBrushless);
  private final RelavtiveEncoder m_encoder = m_climber.getEncoder();
  
  private double targetPoint;
  private boolean moving;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  @Override
  public void periodic() {
    double targetDifference = m_encoder.getPosition()-targetPoint;
    m_climber.set()
    // This method will be called once per scheduler run
  }

  public void setPosition(double target) {
    targetPoint = target;
    moving = true;
  }
}
