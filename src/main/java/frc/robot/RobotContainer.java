// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.WristResetCommand;
import frc.robot.commands.CoralEjectCommand;
import frc.robot.commands.CoralFeederCommand;
import frc.robot.commands.GoToReefCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.GoToReefCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CorallatorSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

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
    private final CorallatorSubsystem m_corallator = new CorallatorSubsystem();
    private final VisionSubsystem m_vision = new VisionSubsystem();
    private final LightsSubsystem m_lightsSubsystem = new LightsSubsystem(m_corallator, m_vision);
	private final ClimberSubsystem m_climber = new ClimberSubsystem();

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    CommandJoystick l_attack3 = new CommandJoystick(0);
    CommandJoystick r_attack3 = new CommandJoystick(1);
    // pathplanner sendable chooser for auto widget i think
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
                                -MathUtil.applyDeadband(
                                        l_attack3.getY() * Math
                                                .abs(l_attack3.getY())
                                                + m_driverController
                                                        .getLeftY()
                                                        * OIConstants.kSecondDriverPower,
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(
                                        l_attack3.getX() * Math
                                                .abs(l_attack3.getX())
                                                + m_driverController
                                                        .getLeftX()
                                                        * OIConstants.kSecondDriverPower,
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(
                                        r_attack3.getX() * Math
                                                .abs(r_attack3.getX()),
                                        OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));
        // build an autochooser. Uses Commands.none() as default option
        autoChooser = getAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private SendableChooser<Command> getAutoChooser() {

        Command autoDropCoralCommand = new InstantCommand(m_corallator::angleReef)
                // .andThen(new InstantCommand(m_elevatorShift::L2))
                .andThen(new GoToReefCommand(m_vision, m_robotDrive))
                .andThen(new GoToReefCommand(m_vision, m_robotDrive).withTimeout(0.025))
                .andThen(new DriveDistanceCommand(m_vision, m_robotDrive))
                .andThen(new RunCommand(m_corallator::outtakeCoral).withTimeout(2))
                .andThen(new InstantCommand(m_corallator::stopCoral));

        NamedCommands.registerCommand("Vision and drop coral", autoDropCoralCommand);

        NamedCommands.registerCommand("Spit Out the Coral", new CoralEjectCommand(m_corallator));
        NamedCommands.registerCommand("Eat the Coral", new CoralFeederCommand(m_corallator));
        NamedCommands.registerCommand("Ready for Intake",
                new InstantCommand(m_elevatorShift::coral_station, m_elevatorShift));
        NamedCommands.registerCommand("Coral Low", new InstantCommand(m_elevatorShift::coral_low, m_elevatorShift));
        NamedCommands.registerCommand("Coral High", new InstantCommand(m_elevatorShift::coral_high, m_elevatorShift)); // duplicates
        NamedCommands.registerCommand("Algae Low", new InstantCommand(m_elevatorShift::algae_low, m_elevatorShift)); // but
                                                                                                                     // who
                                                                                                                     // cares
                                                                                                                     // ?
        NamedCommands.registerCommand("Algae High", new InstantCommand(m_elevatorShift::algae_high, m_elevatorShift));
        NamedCommands.registerCommand("Angle Reef", new InstantCommand(m_corallator::angleReef, m_corallator)); // these
                                                                                                                // are
        NamedCommands.registerCommand("Angle Station", new InstantCommand(m_corallator::angleStation, m_corallator)); // all
                                                                                                                      // self-
        NamedCommands.registerCommand("Angle Algae", new InstantCommand(m_corallator::angleAlgae, m_corallator)); // explanatory
        NamedCommands.registerCommand("Wrist Reset", new WristResetCommand(m_corallator));
        return AutoBuilder.buildAutoChooser();
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
        r_attack3.button(2).whileTrue(new RunCommand(m_robotDrive::setX));
        r_attack3.button(7).onTrue(new InstantCommand(m_robotDrive::zeroHeading));
        // let's elevate
        m_driverController.a().onTrue(new InstantCommand(m_elevatorShift::coral_low));
        m_driverController.b().onTrue(new InstantCommand(m_elevatorShift::algae_low));
        m_driverController.x().onTrue(new InstantCommand(m_elevatorShift::algae_high));
        m_driverController.y().onTrue(new InstantCommand(m_elevatorShift::coral_high));

        // upa nd down :))
        m_driverController.povDown().onTrue(new InstantCommand(m_corallator::angleReef));
        m_driverController.povUp().onTrue(new InstantCommand(m_corallator::angleAlgae));
        m_driverController.povRight().onTrue(new InstantCommand(m_corallator::angleStation));

        // intake and outtake

        m_driverController.rightTrigger()
                .whileTrue(new StartEndCommand(m_corallator::outtakeCoral, m_corallator::stopCoral));
        m_driverController.leftTrigger()
                .whileTrue(new StartEndCommand(m_corallator::intakeCoral, m_corallator::stopCoral));

        // Stuff to make the gyro reset when pressing the "L2" button
        m_driverController.leftStick().onTrue(new WristResetCommand(m_corallator));

        m_driverController.rightStick().onTrue(new InstantCommand(m_elevatorShift::D_stop));

        m_driverController.leftBumper().whileTrue(
        new GoToReefCommand(m_vision, m_robotDrive).andThen(new DriveDistanceCommand(m_vision, m_robotDrive)));
        Trigger climberTrigger = m_driverController.rightBumper();
        climberTrigger.and(m_driverController.axisLessThan(5,-0.5)).onTrue(new InstantCommand(m_climber::climberForward));
        climberTrigger.and(m_driverController.axisGreaterThan(5,0.5)).whileTrue(new InstantCommand(m_climber::climberBackward));
    	}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }

    /**
     * @param pathName Name of a PathPlanner path definition file.
     * @return a Command that will execute the given path.
     */
    private Command fromPathFile(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }
}
