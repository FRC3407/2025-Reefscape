// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CorallatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristResetCommand extends Command {
  /** Creates a new WristResetCommand. */
  private final CorallatorSubsystem m_corallatorSubsystem;
  public WristResetCommand(CorallatorSubsystem corallatorSubsystem) {
    m_corallatorSubsystem = corallatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_corallatorSubsystem.setManualWristSpeed(0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_corallatorSubsystem.angleUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_corallatorSubsystem.isWristSwitchPressed();
  }
}
