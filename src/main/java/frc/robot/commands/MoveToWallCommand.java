// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToWallCommand extends Command 
{

  private final Rev2mDistanceSensor m_distOnboard;
  private final DriveSubsystem m_drivesubsystem;
  /** Creates a new MoveToWallCommand. */
  public MoveToWallCommand(
    Rev2mDistanceSensor distOnboard,
    DriveSubsystem driveSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_distOnboard = distOnboard;
    m_drivesubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivesubsystem.drive(0.1, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivesubsystem.drive(0, 0, 0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_distOnboard.getRange()*2.54/100<=0.7)
  {
    return true;
  }
  else
  {
    return false;
  }
  }
} 
