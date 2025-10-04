// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Serial;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagLookCommand extends Command {
  public final VisionSubsystem visionSubsystem;
  public final DriveSubsystem driveSubsystem;

  /** Creates a new AprilTagLookCommand. */
  public AprilTagLookCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(visionSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Started look command");
    PhotonTrackedTarget target = visionSubsystem.getBestReefTarget();
    if (target != null) {
      // double yaw = target.get;
      // System.out.println(target.getDetectedCorners());
      var corners = target.getDetectedCorners();
      TargetCorner corner = corners.get(0);
      System.out.println(corner);
      double yaw = (corner.x / 320) - 0.5;
      // double yaw = 0;
      System.out.println("yaw: " + yaw);
      if (Math.abs(yaw) < 0.05) {
        System.out.println("i did it");
        driveSubsystem.drive(0, 0, 0, false);

        // this.end(false);
        return;
      }
      driveSubsystem.drive(0, 0, -Math.copySign(0.13, yaw), false);
      // driveSubsystem.drive(0, 0, Math.copySign(0.1, yaw),false,false);

    } else {
      System.out.println("I can't see anything");
      driveSubsystem.drive(0, 0, 0, false);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !visionSubsystem.cameraSeesTargets();
  }
}
