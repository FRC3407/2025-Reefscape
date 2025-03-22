// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.PhotonVersion;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReefCommand extends Command {
  public final VisionSubsystem visionSubsystem;
  public final DriveSubsystem driveSubsystem;

  public static double closeEnoughXDistance = 0.35;
  public static double closeEnoughYDistance = 0.05;
  public static double closeEnoughRotation = 0.1; // about 5.7 degrees
  public static double towardsTagSpeed = 0.1; // TODO: make this a constant later

  public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public Transform3d lastTargetTransform;
  public boolean useLastTransform = false;
  public Pose3d lastPose;

  public float timeSinceAprilTagSeen = 0;

  /** Creates a new GotoAprilTag. */
  public GoToReefCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(visionSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  protected PhotonTrackedTarget getBestTarget() {
    return visionSubsystem.getBestReefTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget target = getBestTarget();
    System.out.println("target: " + target);
    System.out.println("can it see: " + visionSubsystem.cameraSeesTargets());
    if (target != null) {
      timeSinceAprilTagSeen = 0;
      Transform3d camToTarget = target.getBestCameraToTarget();

      double yaw = camToTarget.getRotation().getZ();
      System.out.println("Yaw: " + yaw);
      double rotationSpeed = 0.5;

      double movementX = towardsTagSpeed * camToTarget.getX();
      double movementY = towardsTagSpeed * camToTarget.getY();

      double newYaw = Math.copySign(Math.PI - Math.abs(yaw), yaw);

      if (target.getBestCameraToTarget().getX() < closeEnoughXDistance) {
        movementX = 0;
        movementY *= 3.0;
      }

      driveSubsystem.drive(movementX, movementY, -rotationSpeed * newYaw / Math.max(0.5, camToTarget.getX() * 4.0),
          false);
      lastTargetTransform = camToTarget;
      useLastTransform = true;
      // AprilTagFieldLayout.loadFromResource("")
      lastPose = PhotonUtils.estimateFieldToRobotAprilTag(
        camToTarget,
        fieldLayout.getTagPose(target.fiducialId).get(),
        camToTarget
      );
    } else {
      driveSubsystem.drive(0, 0, 0, false);
      timeSinceAprilTagSeen += 0.02;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
    System.out.println("goto april tag ended. int: " + interrupted);
    System.out.println("last target transform: " + lastTargetTransform);
    if (!interrupted) {
      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    PhotonTrackedTarget target = getBestTarget();
    if (target != null) {
      timeSinceAprilTagSeen = 0;
      Transform3d camToTarget = target.getBestCameraToTarget();

      double yaw = camToTarget.getRotation().getZ();
      double newYaw = Math.copySign(Math.PI - Math.abs(yaw), yaw);

      if (camToTarget.getX() < closeEnoughXDistance &&
          Math.abs(camToTarget.getY()) < closeEnoughYDistance &&
          Math.abs(newYaw) < closeEnoughRotation) {
        System.out.println("I did it :)");
        driveSubsystem.resetOdometry(lastPose.toPose2d());
        return true;
      }
    }
    if (timeSinceAprilTagSeen >= 0.3) {
      System.out.println("I can't find it :(");
      return true;
    }
    return false;
  }
}
