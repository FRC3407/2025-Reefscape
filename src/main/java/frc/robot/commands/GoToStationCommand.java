// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GoToStationCommand extends GoToReefCommand {

  public GoToStationCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    super(visionSubsystem, driveSubsystem);
  }

  @Override
  protected PhotonTrackedTarget getBestTarget() {
    return visionSubsystem.getBestCoralStationTarget();
  }

}