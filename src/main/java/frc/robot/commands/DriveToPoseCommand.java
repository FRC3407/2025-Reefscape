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

    protected Pose2d goalPose;
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
    abstract protected Pose2d getGoalPose();

    @Override
    public void initialize() {
        goalPose = getGoalPose();
        xPidController.setGoal(0.0);
        yPidController.setGoal(0.0);
        headingPidController.setGoal(0.0);
    }

    @Override
    public void execute() {
        if (goalPose != null) {
            Transform2d toGoal = goalPose.minus(driveSubsystem.getPose());
            ChassisSpeeds speeds = new ChassisSpeeds(
                    xPidController.calculate(-toGoal.getX()),
                    yPidController.calculate(-toGoal.getY()),
                    headingPidController.calculate(-toGoal.getRotation().getRadians()));
            driveSubsystem.driveRobotRelative(speeds, feedforwards);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
        goalPose = null;
    }

    @Override
    public boolean isFinished() {
        if (goalPose != null) {
            Pose2d relativePose = driveSubsystem.getPose().relativeTo(goalPose);
            double distanceToGoal = relativePose.getTranslation().getNorm();
            double diffToAngle = Math.abs(relativePose.getRotation().getRadians());
            return distanceToGoal <= CLOSE_DISTANCE && diffToAngle <= CLOSE_ANGLE;
        } else {
            return true;
        }
    }
}
