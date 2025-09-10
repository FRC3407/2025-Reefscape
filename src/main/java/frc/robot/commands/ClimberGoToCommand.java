// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberGoToCommand extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final double setPoint;

    public ClimberGoToCommand(double setPoint, ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.setPoint = setPoint;
    }

    @Override
    public void initialize() {
        climberSubsystem.setClimberGoal(setPoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
