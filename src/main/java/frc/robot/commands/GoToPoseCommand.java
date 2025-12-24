// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToPoseCommand extends Command {
  public final DriveSubsystem m_robotDrive;
  public Pose2d targetPose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
  
  public final double positionThreshold = 0.0025; // in meters probably
  public final double rotationThreshold = 0.02; // in degrees bc i used getDegrees() for the pid controller
  
  public PIDController xPID = new PIDController(0.6, 0.02, 0.00);
  public PIDController yPID = new PIDController(0.6, 0.02, 0.00);
  public PIDController rPID = new PIDController(0.010, 0.002, 0);

  /** Creates a new GoToPoseCommand. */
  public GoToPoseCommand(Pose2d target, DriveSubsystem drive) {
    m_robotDrive = drive;
    targetPose = target;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPID.reset();
    yPID.reset();
    rPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_robotDrive.getPose();
    Pose2d delta = currentPose.relativeTo(targetPose);
    System.out.println(delta);
    
    m_robotDrive.drive(
      Math.min(0.4,xPID.calculate(delta.getX())),
      Math.min(0.4,yPID.calculate(delta.getY())),
      Math.min(0.4,rPID.calculate(delta.getRotation().getDegrees())),
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xPID.reset();
    yPID.reset();
    rPID.reset();
    m_robotDrive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = m_robotDrive.getPose();
    Pose2d delta = currentPose.relativeTo(targetPose);

    return Math.hypot(delta.getX(), delta.getY()) < positionThreshold && Math.abs(delta.getRotation().getDegrees()) < rotationThreshold;
  }
}
