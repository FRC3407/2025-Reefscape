// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAngleCommand extends Command {
  /** Creates a new TurnToAngleCommand. */
  private final DriveSubsystem m_DriveSubsystem;
  private final Double m_TargetAngle;
  private double goalAngle;
  public TurnToAngleCommand(DriveSubsystem driveSubsystem, double targetAngle) {
    m_DriveSubsystem = driveSubsystem;
    m_TargetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalAngle = m_DriveSubsystem.getHeading() + m_TargetAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleDifference = goalAngle - m_DriveSubsystem.getHeading();
    m_DriveSubsystem.drive(0, 0, -(0.01 * angleDifference), false);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angleDifference = goalAngle - m_DriveSubsystem.getHeading();
    System.out.println(m_DriveSubsystem.isConnected() + "  " + m_DriveSubsystem.isCalibrating() + "  " + Math.round(angleDifference*100.)/100. + "  " + m_DriveSubsystem.getTurnRate());
    return Math.abs(angleDifference) < 2;
  }
}
