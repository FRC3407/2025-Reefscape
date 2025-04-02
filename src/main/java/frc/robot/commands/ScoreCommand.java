// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CorallatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCommand extends Command {
  private final CorallatorSubsystem corallatorSubsystem;
  private final Timer timer = new Timer();

  /** Creates a new ScoreCommand. */
  public ScoreCommand(CorallatorSubsystem corallatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.corallatorSubsystem = corallatorSubsystem;
    addRequirements(corallatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    corallatorSubsystem.outtakeCoral();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    corallatorSubsystem.stopCoral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);
  }
}
