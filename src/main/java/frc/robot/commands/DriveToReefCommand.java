package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToReefCommand extends DriveToPoseCommand {

    public final VisionSubsystem visionSubsystem;
    private final Transform2d offsetTransform;

    public DriveToReefCommand(ReefOffset reefOffset, VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
        super(driveSubsystem);
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);
        offsetTransform = reefOffset.transform;
    }

    protected PhotonTrackedTarget getBestTarget() {
        return visionSubsystem.getBestReefTarget();
    }

    @Override
    protected Pose2d getGoalPose() {
        PhotonTrackedTarget target = getBestTarget();
        if (target != null) {
            Transform2d camToTarget = toTransform2d(target.getBestCameraToTarget());
            Pose2d robotPose = driveSubsystem.getPose();
            return robotPose.plus(camToTarget).plus(offsetTransform);
        } else {
            System.err.println("DriveToReefCommand: no target found");
            return null;
        }
    }

    private Transform2d toTransform2d(Transform3d t3d) {
        return new Transform2d(t3d.getTranslation().toTranslation2d(), t3d.getRotation().toRotation2d());
    }

    public static enum ReefOffset {
        LEFT(0.050, -0.017),
        RIGHT(0.050, 0.017),
        CENTER(0.050, 0.000);

        public final Transform2d transform;

        private ReefOffset(double x, double y) {
            this.transform = new Transform2d(x, y, Rotation2d.fromDegrees(-180));
        }
    }

}
