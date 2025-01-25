// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForwardUltrasonicCommand extends Command {
  /** Creates a new DriveForwardUltrasonicCommand. */

  Timer timer;
  private final DriveSubsystem m_DriveSubsystem;
  private final PIDController pidController;

  public DriveForwardUltrasonicCommand(DriveSubsystem driveSubsystem) {
    m_DriveSubsystem = driveSubsystem;
    pidController = new PIDController(0.05, 0, 0);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveSubsystem.drive(pidController.calculate(m_DriveSubsystem.getDistance(),1.0), 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0, 0, 0, false);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(5)){
      System.out.println("finished command");
      return true;
    }
    return false; //(pidController.atSetpoint());
  }
}
