// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Serial;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  public PhotonCamera camera;

  // public NetworkTable table;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    System.out.println("hello im vision");
    camera = new PhotonCamera("Arducam");

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

}
