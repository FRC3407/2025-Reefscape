// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveDistanceCommand extends Command {
  public final DriveSubsystem driveSubsystem;

  // In meters
  public static double closeEnoughDistance = 0.1;

  public double x;
  public double y;
  public double speed;
  public Pose2d startingPose;

  /**
   * Creates a new DriveDistanceCommand
   * @param driveSubsystem
   * @param x X (forwards-backwards) meters
   * @param y Y (left-right) meters
   */
  public DriveDistanceCommand(DriveSubsystem driveSubsystem, double x, double y, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem=driveSubsystem;
    this.x=x;
    this.y=y;
    this.speed=speed;
    this.startingPose = driveSubsystem.getPose();
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
    Pose2d relativePose = currentPose.relativeTo(startingPose);
    if (Math.sqrt(relativePose.getX()*relativePose.getX() + relativePose.getY()*relativePose.getY())>closeEnoughDistance) {
        driveSubsystem.drive(
            Math.copySign(speed, x),
            Math.copySign(speed, y),
            0,
            false
        );
    }
    //loser loser hahahahahaha wheee ooga booga hghjhygfghjkhgfghjhgfdfghjuytrerfghjkiuytrertyuiku sorry i needded to do this
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
