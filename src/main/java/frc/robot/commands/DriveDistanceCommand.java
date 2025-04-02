// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveDistanceCommand extends Command {
  public final DriveSubsystem m_robotDrive;
  public final VisionSubsystem m_vision;

  public static double distanceThreshold = 0.005;

  public double moveX;
  public double moveY;

  public Pose2d startPose;
  public Pose2d destinationPose;
  /** Creates a new DriveDistanceCommand. */
  // public DriveDistanceCommand(double x, double y, DriveSubsystem drive) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   System.out.println("drive dist x="+x+" y="+y);
  //   m_robotDrive = drive;
  //   moveX=x;
  //   moveY=y;
  //   addRequirements(drive);
  // }
  public DriveDistanceCommand(VisionSubsystem vision, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    System.out.println("drive dist vision");
    m_robotDrive = drive;
    m_vision=vision;
    addRequirements(drive,vision);
  }

  public Pose2d getDelta() {
    return destinationPose.relativeTo(m_robotDrive.getPose());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_robotDrive.jvireavo;
    startPose = m_robotDrive.getPose();
    moveX=m_vision.lastTransformStash.getX();
    moveY=m_vision.lastTransformStash.getY();
    destinationPose = m_robotDrive.getPose().transformBy(new Transform2d(moveX,moveY,new Rotation2d(0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed=0.08;
    Pose2d delta=getDelta();
    double x=(Math.abs(delta.getX())<0.001) ? 0.0 : Math.copySign(speed, delta.getX());
    double y=(Math.abs(delta.getY())<0.001) ? 0.0 : Math.copySign(speed, delta.getY());
    System.out.println("im movin x=" + x + " y=" + y);
    m_robotDrive.drive(
      x, 
      y, 
      0, 
      false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("drive distance ended. int:" + interrupted);
    m_robotDrive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d delta=getDelta();
    System.out.println("delta="+delta);
    double distance = Math.sqrt(delta.getX()*delta.getX() + delta.getY()*delta.getY());
    System.out.println("dist="+distance);
    return distance<distanceThreshold;
  }
}
