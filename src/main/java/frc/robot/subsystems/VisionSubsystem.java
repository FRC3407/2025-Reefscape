// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    public static List<Integer> redReefTags = Arrays.asList(6, 7, 8, 9, 10, 11);
    public static List<Integer> redCoralStationTags = Arrays.asList(12, 13);
    public static List<Integer> blueReefTags = Arrays.asList(17, 18, 19, 20, 21, 22);
    public static List<Integer> blueCoralStationTags = Arrays.asList(1, 2);

    public List<PhotonPipelineResult> lastResults;
    public float timeSinceLastResults = 0;

    private PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private boolean isTargetVisible = false;

    private final DriveSubsystem m_robotDrive;

    // public NetworkTable table;
    /** Creates a new VisionSubsystem. */
    public VisionSubsystem(DriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        Transform3d cameraTransform = null; // TODO: calculate camera offset within robot.
        photonEstimator = new PhotonPoseEstimator(tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraTransform);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public EstimatedRobotPose getEstimatedGlobalPose(PhotonCamera cam, PhotonPoseEstimator estimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (PhotonPipelineResult pipelineResult : cam.getAllUnreadResults()) {
            visionEst = estimator.update(pipelineResult);
        }
        return visionEst.orElse(null);
    }

    @Override
    public void periodic() {
        EstimatedRobotPose poseEstimate = getEstimatedGlobalPose(camera, photonEstimator);
        isTargetVisible = (poseEstimate != null);
        if (isTargetVisible) {
            m_robotDrive.addVisionMeasurement(poseEstimate.estimatedPose.toPose2d(), poseEstimate.timestampSeconds);
        }
        timeSinceLastResults += 1.0 / 50.0;
    }

    private List<Integer> getReefTags() {
        return DriverStation.getAlliance().get() == Alliance.Red ? redReefTags : blueReefTags;
    }

    private List<Integer> getCoralStationTags() {
        return DriverStation.getAlliance().get() == Alliance.Red ? redCoralStationTags : blueCoralStationTags;
    }

    public PhotonTrackedTarget getBestReefTarget() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.size() > 0) {
            lastResults = results;
            timeSinceLastResults = 0;
        }
        if (results.size() == 0) { // Skip if no results
            if (timeSinceLastResults <= 1.0 / 9.0) // Use last results if they were recent enough (1/9th of a second)
                results = lastResults;
            else
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

    public boolean cameraSeesTargets() {
        return isTargetVisible;
    }

}
