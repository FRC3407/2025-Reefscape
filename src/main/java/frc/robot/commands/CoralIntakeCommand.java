// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CorallatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeCommand extends Command {
	/** Creates a new CoralEjectCommand. */
	private final CorallatorSubsystem m_corallator;
	private final Timer waitTimer;

	public CoralIntakeCommand(CorallatorSubsystem corallatorSub) {
		m_corallator = corallatorSub;
		addRequirements(m_corallator);

		waitTimer = new Timer();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_corallator.intakeCoral();
		waitTimer.reset();
		waitTimer.start();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_corallator.stopCoral();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return waitTimer.hasElapsed(1.5);
	}
}
