// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Serial;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  public static List<Integer> redReefTags = Arrays.asList(6, 7, 8, 9, 10, 11);
  public static List<Integer> redCoralStationTags = Arrays.asList(12, 13);
  public static List<Integer> blueReefTags = Arrays.asList(17, 18, 19, 20, 21, 22);
  public static List<Integer> blueCoralStationTags = Arrays.asList(1, 2);

  public PhotonCamera camera;

  // public NetworkTable table;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    System.out.println("hello im vision");
    camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

  }

  @Override
  public void periodic() {
    // System.out.println("i see");
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      // System.out.println("OH MY GOD ITS AN APRILTAG!");
      // System.out.println(result.getTargets());
      PhotonTrackedTarget target = result.getBestTarget();

    }

  }

  private List<Integer> getReefTags() {
    return DriverStation.getAlliance().get() == Alliance.Red ? redReefTags : blueReefTags;
  }

  private List<Integer> getCoralStationTags() {
    return DriverStation.getAlliance().get() == Alliance.Red ? redCoralStationTags : blueCoralStationTags;
  }

  public PhotonTrackedTarget getBestReefTarget() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.size() == 0) { // Skip if no results
      return null;
    }
    PhotonPipelineResult result = results.get(0);
    if (!result.hasTargets()) {
      return null;
    }
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (getReefTags().indexOf(target.getFiducialId()) != -1) {
        return target;
      }
    }
    return null;
  }

  public PhotonTrackedTarget getBestCoralStationTarget() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.size() == 0) { // Skip if no results
      return null;
    }
    PhotonPipelineResult result = results.get(0);
    if (!result.hasTargets()) {
      return null;
    }
    for (PhotonTrackedTarget target : result.getTargets()) {
      if (getCoralStationTags().indexOf(target.getFiducialId()) != -1) {
        return target;
      }
    }
    return null;
  }

}
