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

    private final double MAX_SPEED = DriveConstants.kMaxSpeedMetersPerSecond;
    private final double MAX_ACCEL = AutoConstants.kMaxAccelerationMetersPerSecondSquared;
    private final double MAX_ROTATE = DriveConstants.kMaxAngularSpeed;
    private final double MAX_ANGULAR_ACCEL = AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared;
    private final double CLOSE_DISTANCE = 0.2;

    private final DriveSubsystem driveSubsystem;

    private ProfiledPIDController xController = new ProfiledPIDController(10, 0, 0.3,
            new Constraints(MAX_SPEED, MAX_ACCEL)); // Forward/back
    private ProfiledPIDController yController = new ProfiledPIDController(10, 0, 0.3,
            new Constraints(MAX_SPEED, MAX_ACCEL)); // Left/right
    private ProfiledPIDController headingController = new ProfiledPIDController(7, 0, 0,
            new Constraints(MAX_ROTATE, MAX_ANGULAR_ACCEL)); // rotation

    private Pose2d goalPose;
    private DriveFeedforwards feedforwards = null;

    public DriveToPoseCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        xController.setGoal(0.0);
        yController.setGoal(0.0);
        headingController.setGoal(0.0);
    }

    /**
     * @return goal pose for this command.
     */
    abstract public Pose2d getGoalPose();

    @Override
    public void initialize() {
        goalPose = getGoalPose();
    }

    @Override
    public void execute() {
        Transform2d toGoal = goalPose.minus(driveSubsystem.getPose());
        ChassisSpeeds speeds = new ChassisSpeeds(
                xController.calculate(-toGoal.getX()),
                yController.calculate(-toGoal.getY()),
                headingController.calculate(-toGoal.getRotation().getRadians()));
        driveSubsystem.driveRobotRelative(speeds, feedforwards);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        double distanceToGoal = driveSubsystem.getPose().relativeTo(goalPose).getTranslation().getNorm();
        return distanceToGoal <= CLOSE_DISTANCE;
    }
}
