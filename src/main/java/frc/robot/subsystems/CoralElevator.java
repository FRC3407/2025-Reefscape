// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class CoralElevator extends SubsystemBase {

    private final SparkMax m_elevator = new SparkMax(ElevatorCanId, MotorType.kBrushless); // object variables
    private final RelativeEncoder m_encoder = m_elevator.getEncoder(); // get the encoder value from motor
    private final SparkLimitSwitch m_switch1 = m_elevator.getForwardLimitSwitch();
    private final PIDController m_control = new PIDController(P_value, I_value, D_value); // gets the pid values from
                                                                                          // the
                                                                                          // constants subsystem
    private double set_point;

    /** Creates a new CoralElevator. */
    public CoralElevator() {
        m_encoder.setPosition(0); // sets the value of encoder as 0
        m_elevator.configure(Configs.ElevatorConfig.m_elavator, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    private void set_position(double height) {
        set_point = height;

    }

    public void stop() {
        set_point = m_encoder.getPosition();
    }

    public void L1() {
        set_position(level_1);
    } // these are all methods

    public void L2() {
        set_position(level_2);
    } // void id returning nothing

    public void L3() {
        set_position(level_3);
    }

    public void L4() {
        set_position(level_4);
  }

    public void D_stop() {
        set_position(d_stop);
    }

    public void coral_station() {
        set_position(coral_s);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (set_point == 0) {
            m_elevator.set(0.5);
            m_encoder.setPosition(0);
        } else {
            m_elevator.set(m_control.calculate(m_encoder.getPosition(), set_point));// called 50 times a second
            SmartDashboard.putNumber("Elevator Height", m_encoder.getPosition());
            SmartDashboard.putNumber("Set Point", set_point);
        }
    }
}
