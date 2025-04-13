package frc.robot.commands;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

/**
 * Command that moves the {@code DriveSubsystem} to a specific {@code Pose2d}.
 */
abstract public class DriveToPoseCommand extends Command {

    protected final double MAX_SPEED = DriveConstants.kMaxSpeedMetersPerSecond;
    protected final double MAX_ACCEL = AutoConstants.kMaxAccelerationMetersPerSecondSquared;
    protected final double MAX_ROTATE = DriveConstants.kMaxAngularSpeed;
    protected final double MAX_ANGULAR_ACCEL = AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared;
    protected final double CLOSE_DISTANCE = 0.2; // in meters
    protected final double CLOSE_ANGLE = 0.15; // in radians

    private ProfiledPIDController xPidController = new ProfiledPIDController(10, 0, 0.3,
            new Constraints(MAX_SPEED, MAX_ACCEL)); // Forward/back
    private ProfiledPIDController yPidController = new ProfiledPIDController(10, 0, 0.3,
            new Constraints(MAX_SPEED, MAX_ACCEL)); // Left/right
    private ProfiledPIDController headingPidController = new ProfiledPIDController(7, 0, 0,
            new Constraints(MAX_ROTATE, MAX_ANGULAR_ACCEL)); // rotation

    protected final DriveSubsystem driveSubsystem;

    protected Pose2d targetPose;
    protected DriveFeedforwards feedforwards = null;

    public DriveToPoseCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    /**
     * Return the pose we want to drive to, or {@code null} if no pose is available.
     * 
     * @return goal pose for this command or {@code null}.
     */
    abstract protected Pose2d makeTargetPose();

    @Override
    public void initialize() {
        targetPose = makeTargetPose();
        xPidController.setGoal(0.0);
        yPidController.setGoal(0.0);
        headingPidController.setGoal(0.0);
    }

    @Override
    public void execute() {
        if (targetPose != null) {
            Transform2d toTarget = targetPose.minus(driveSubsystem.getPose());
            ChassisSpeeds speeds = new ChassisSpeeds(
                    xPidController.calculate(-toTarget.getX()),
                    yPidController.calculate(-toTarget.getY()),
                    headingPidController.calculate(-toTarget.getRotation().getRadians()));
            driveSubsystem.driveRobotRelative(speeds, feedforwards);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
        targetPose = null;
    }

    @Override
    public boolean isFinished() {
        if (targetPose != null) {
            Pose2d relativePose = driveSubsystem.getPose().relativeTo(targetPose);
            double distanceToTarget = relativePose.getTranslation().getNorm();
            double diffToAngle = Math.abs(relativePose.getRotation().getRadians());
            return distanceToTarget <= CLOSE_DISTANCE && diffToAngle <= CLOSE_ANGLE;
        } else {
            return true;
        }
    }
}
