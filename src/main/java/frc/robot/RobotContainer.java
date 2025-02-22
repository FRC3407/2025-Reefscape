// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import frc.robot.subsystems.CorallatorSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralElevator m_elevatorShift = new CoralElevator();
  private final CorallatorSubsystem m_Corallator = new CorallatorSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  Joystick l_attack3 = new Joystick(0);
  Joystick r_attack3 = new Joystick(1);
  //pathplanner sendable chooser for auto widget i think
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(l_attack3.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(l_attack3.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(r_attack3.getX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    //build an autochooser. Uses Commands.none() as default option
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(r_attack3, 2)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    /*m_driverController.x().onTrue(new TurnToAngleCommand(m_robotDrive, -45));
    m_driverController.a().onTrue(new TurnToAngleCommand(m_robotDrive, 45));*/
    m_driverController.povDown().onTrue(new InstantCommand(m_Corallator::angleDown));
    m_driverController.povUp().onTrue(new InstantCommand(m_Corallator::angleUp));

    // intake and outtake
    JoystickButton unbing = new JoystickButton(r_attack3, 1);
    unbing.onTrue(new InstantCommand(m_Corallator::outtakeCoral));

    JoystickButton bing = new JoystickButton(r_attack3, 2);
    bing.onTrue(new InstantCommand(m_Corallator::intakeCoral));

    // upa nd down
    JoystickButton coralatorupper = new JoystickButton(l_attack3, 3);
    coralatorupper.onTrue(new InstantCommand(m_Corallator::angleUp));

    JoystickButton coralatordowner = new JoystickButton(l_attack3, 2);
    coralatordowner.onTrue(new InstantCommand(m_Corallator::angleDown));

    //new stuff to make the gyro reset when pressing the "L2" button
    new JoystickButton(r_attack3, 7)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    new JoystickButton(r_attack3, 6)
        .onTrue(new InstantCommand(
            () -> m_elevatorShift.coral_station(),
            m_elevatorShift));
        new JoystickButton(r_attack3, 5)
        .onTrue(new InstantCommand(
            () -> m_elevatorShift.L1(),
            m_elevatorShift));
            new JoystickButton(r_attack3, 4)
        .onTrue(new InstantCommand(
            () -> m_elevatorShift.L2(),
            m_elevatorShift));
            new JoystickButton(r_attack3, 3)
        .onTrue(new InstantCommand(
            () -> m_elevatorShift.L3(),
            m_elevatorShift));
          //  new JoystickButton(r_attack3, 8)
       // .onTrue(new InstantCommand(
          //  () -> m_elevatorShift.L4(),
           // m_elevatorShift));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    /* 
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
*/

    }
}
