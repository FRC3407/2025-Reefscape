// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveDistanceCommand extends Command {
  public final DriveSubsystem driveSubsystem;

  // In meters
  public static double closeEnoughDistance = 0.1;

  public double speed;
  public Pose2d startingPose;
  public Pose2d targetPose;

  /**
   * Creates a new DriveDistanceCommand
   * @param driveSubsystem
   * @param x X (forwards-backwards) meters
   * @param y Y (left-right) meters
   */
  public DriveDistanceCommand(DriveSubsystem driveSubsystem, double x, double y, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem=driveSubsystem;
    this.speed=speed;
    this.startingPose = driveSubsystem.getPose();
    Transform2d offset = new Transform2d(
      new Translation2d(x,y),
      Rotation2d.fromDegrees(0)
    );
    this.targetPose = this.startingPose.plus(offset);
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = driveSubsystem.getPose();
    Pose2d relativePose = currentPose.relativeTo(targetPose);
    double distance = Math.sqrt(Math.pow(relativePose.getX(),2) + Math.pow(relativePose.getY(),2));

    driveSubsystem.drive(
        relativePose.getX()/distance*speed,
        relativePose.getY()/distance*speed,
        0,
        false
    );

    //loser loser hahahahahaha wheee ooga booga hghjhygfghjkhgfghjhgfdfghjuytrerfghjkiuytrertyuiku sorry i needded to do this
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = driveSubsystem.getPose();
    Pose2d relativePose = currentPose.relativeTo(targetPose);
    double distance = Math.sqrt(Math.pow(relativePose.getX(),2) + Math.pow(relativePose.getY(),2));
    return distance < closeEnoughDistance;
  }
}
