// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GotoAprilTagCommand extends Command {
  public final VisionSubsystem visionSubsystem;
  public final DriveSubsystem driveSubsystem;

  public float timeSinceAprilTagSeen = 0;

  /** Creates a new GotoAprilTag. */
  public GotoAprilTagCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem=visionSubsystem;
    this.driveSubsystem=driveSubsystem;
    addRequirements(visionSubsystem,driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = visionSubsystem.camera.getLatestResult();
    if (result.hasTargets()) {
      timeSinceAprilTagSeen = 0;
      System.out.println("I found a target");
      PhotonTrackedTarget target = result.getBestTarget();
      Transform3d camToTarget = target.getBestCameraToTarget();
      double speed = 0.001; // TODO: make this a constant later
      driveSubsystem.drive(speed*camToTarget.getX(), speed*camToTarget.getY(), 0, false);

    } else {
      timeSinceAprilTagSeen += 1/50;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    PhotonPipelineResult result = visionSubsystem.camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (Math.abs(target.getBestCameraToTarget().getX())<0.1) {
        return true;
      }
    }
    if (timeSinceAprilTagSeen >= 10) {
      return true
    }
    return false;
  }
}
