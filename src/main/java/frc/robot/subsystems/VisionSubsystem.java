package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private boolean isTargetVisible = false;
    private final DriveSubsystem driveSubsystem;
    public final NetworkTableEntry apriltagsVisibleDisplay = SmartDashboard.getEntry("VisionSubsystem/AprilTagsVisible");
    public final NetworkTableEntry periodicTimeDisplay = SmartDashboard.getEntry("VisionSubsystem/periodicTime");
    public final List<VisionCamera> cameraList = new ArrayList<>();
    protected final AprilTagFieldLayout tagLayout;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.tagLayout = AprilTagFieldLayout.loadField(VisionConstants.kFieldLayout);
        cameraList.add(new VisionCamera("Arducam_OV9281_USB_Camera", new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(0.0), // left or right from center
                        Units.inchesToMeters(8.0), // forward from robot center
                        Units.inchesToMeters(12.0)), // distance up from the floor
                new Rotation3d(
                        Rotation2d.fromDegrees(0).getRadians(), // roll
                        Rotation2d.fromDegrees(0.0).getRadians(), // pitch
                        Rotation2d.fromDegrees(0).getRadians())))); // yaw
    }

    @Override
    public void periodic() {
        final double time = Timer.getFPGATimestamp();
        isTargetVisible = false;
        for (VisionCamera visionCamera : cameraList) {
            EstimatedRobotPose poseEstimate = visionCamera.getEstimatedRobotPose();
            if (poseEstimate != null) {
                isTargetVisible = true;
                if (poseEstimate.timestampSeconds != visionCamera.prevTimestamp) {
                    driveSubsystem.addVisionMeasurement(
                            poseEstimate.estimatedPose.toPose2d(),
                            poseEstimate.timestampSeconds);
                }
                visionCamera.prevTimestamp = poseEstimate.timestampSeconds;
            }
        }
        apriltagsVisibleDisplay.setBoolean(isTargetVisible);
        periodicTimeDisplay.setInteger(Math.round(1000*(Timer.getFPGATimestamp()-time)));
    }

    /**
     * @return whether any AprilTag is visible.
     */
    public boolean cameraSeesTargets() {
        return isTargetVisible;
    }

    /**
     * Find the best {@code AprilTag} currently visible for a given list of IDs.
     * If no fiducial IDs are specified, then return any possible tag.
     *
     * @param fiducialIDs optional list of tag IDs.
     * @return the best visible {@code AprilTag} for any of the fiducialIDs.
     */
    public AprilTag getBestTag(Integer... fiducialIDs) {
        AprilTag aprilTag = null;
        for (VisionCamera visionCamera : cameraList) {
            aprilTag = visionCamera.getBestTag(fiducialIDs);
            if (aprilTag != null) { break; }
        }
        return aprilTag;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    public class VisionCamera {

        final PhotonCamera camera;
        final Transform3d cameraTransform;
        final PhotonPoseEstimator poseEstimator;
        double prevTimestamp;

        public VisionCamera(String cameraName, Transform3d cameraTransform) {
            this(new PhotonCamera(cameraName),
                    cameraTransform,
                    new PhotonPoseEstimator(tagLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            cameraTransform));
            this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        VisionCamera(PhotonCamera camera, Transform3d cameraTransform, PhotonPoseEstimator poseEstimator) {
            this.camera = camera;
            this.cameraTransform = cameraTransform;
            this.poseEstimator = poseEstimator;
        }

        /**
         * @return the estimated field-centric pose of the robot, or
         * {@code null}.
         */
        EstimatedRobotPose getEstimatedRobotPose() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (PhotonPipelineResult pipelineResult : camera.getAllUnreadResults()) {
                List<PhotonTrackedTarget> targetList = makeGoodTargetList(pipelineResult, null);
                if (!targetList.isEmpty()) {
                    visionEst = poseEstimator.update(pipelineResult);
                }
            }
            return visionEst.orElse(null);

        }

        /**
         * @return True if the camera is actively sending frame data, false
         * otherwise.
         */
        public boolean isConnected() {
            return camera.isConnected();
        }

        /**
         * @return the best visible {@code AprilTag} for any of the fiducialIDs.
         */
        public AprilTag getBestTag(Integer... fiducialIDs) {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            if (results == null || results.isEmpty()) {
                return null;
            }
            return getBestTag(results.get(0), fiducialIDs);
        }

        /**
         * @return the best {@code AprilTag} or {@code null}.
         */
        AprilTag getBestTag(PhotonPipelineResult pipelineResult, Integer... fiducialIDs) {
            if (!pipelineResult.hasTargets()) {
                return null;
            }
            List<PhotonTrackedTarget> targetList = makeGoodTargetList(pipelineResult,
                    (t1, t2) -> Double.compare(t1.poseAmbiguity, t2.poseAmbiguity));
            for (PhotonTrackedTarget target : targetList) {
                if (isFiducialListMatch(target, fiducialIDs)) {
                    return toAprilTag(pipelineResult, target);
                }
            }
            return null;
        }

        /**
         * @return whether the observed target is on a list of IDs. If the ID
         * list is empty, then match any target.
         */
        boolean isFiducialListMatch(PhotonTrackedTarget target, Integer... fiducialIDs) {
            if (fiducialIDs == null || fiducialIDs.length == 0) {
                return true;
            }
            for (Integer id : fiducialIDs) {
                if (target.getFiducialId() == id) {
                    return true;
                }
            }
            return false;
        }

        /**
         * Convert a pipeline result and target to an {@code AprilTag} object
         * with a field-relative {@code Pose3d}.
         */
        AprilTag toAprilTag(PhotonPipelineResult result, PhotonTrackedTarget target) {
            Optional<EstimatedRobotPose> visionEst = poseEstimator.update(result);
            if (!visionEst.isPresent()) {
                return null;
            }
            return makeAprilTag(target.getFiducialId(), 
                visionEst.get().estimatedPose, 
                target.getBestCameraToTarget());
        }

        AprilTag makeAprilTag(int fiducialId, Pose3d fieldToRobotPose, Transform3d robotToTag) {
            Pose3d aprilTagPose = fieldToRobotPose.transformBy(robotToTag);
            return new AprilTag(fiducialId, aprilTagPose);
        }

        /**
         * @return whether this target should be excluded from vision
         * processing.
         */
        boolean isBadTarget(PhotonTrackedTarget target) {
            // TODO: This method might be rewritten to add different quality checks.
            return target.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity;
        }

        /**
         * @return List of all {@code PhotonTrackedTarget} objects that are not "bad".
         */
        List<PhotonTrackedTarget> makeGoodTargetList(PhotonPipelineResult pipelineResult,
                                                              Comparator<PhotonTrackedTarget> targetComparator) {
            List<PhotonTrackedTarget> targetList = new ArrayList<>(pipelineResult.getTargets());
            List<PhotonTrackedTarget> badTargets = pipelineResult.getTargets().stream()
                    .filter(this::isBadTarget).toList();
            targetList.removeAll(badTargets);
            if (!targetList.isEmpty() && targetComparator != null) {
                Collections.sort(targetList, targetComparator);
            }
            return targetList;
        }
    }
}