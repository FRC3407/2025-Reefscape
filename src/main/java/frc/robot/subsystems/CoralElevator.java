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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class CoralElevator extends SubsystemBase {

	private final SparkMax m_elevator = new SparkMax(ElevatorCanId, MotorType.kBrushless); // object variables
	private final RelativeEncoder m_encoder = m_elevator.getEncoder(); // get the encoder value from motor
	private final SparkLimitSwitch m_switch1 = m_elevator.getForwardLimitSwitch();
	private final SparkLimitSwitch m_switch0 = m_elevator.getReverseLimitSwitch();
	private final PIDController m_control = new PIDController(P_value, I_value, D_value); // gets the pid values from
																							// the constants subsystem
	private double set_point;

	/** Creates a new CoralElevator. */

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

	/** Go to lower coral height (L1) */
	public void coral_low() {
		get_position(L1);
	} // these are all methods

	/** Go to higher coral height (L3) */
	public void coral_high() {
		set_position(L3);
	}
	
	/** Go to lower algae height (L2) */
	public void algae_low() {
		set_position(L2);
	}

	/** Go to higher algae height (L4) */
	public void algae_high() {	
		set_position(L4);
	} // THIS FUNCTION IS UNUSED BUT DON'T DELETE IT BECAUSE IT MIGHT BE USEFUL AT SOME POINT OR ANOTHER OK THANK YOU

	public void D_stop() {
		set_position(d_stop);
	}

	public void coral_station() {
		set_position(coral_s);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		SmartDashboard.putNumber("Elevator Height", m_encoder.getPosition());
		SmartDashboard.putNumber("Set Point", set_point);
		SmartDashboard.putString("Using PID", "NO");
		SmartDashboard.putBoolean("Forward Limit Switch Status", m_switch1.isPressed());
		SmartDashboard.putBoolean("Reverse Limit Switch Status", m_switch0.isPressed());
		if (set_point == 0) {
			m_elevator.set(0.5);
			m_encoder.setPosition(0);
		} else {

			double motor_speed = MathUtil.clamp(-0.1 + m_control.calculate(m_encoder.getPosition(), set_point), -1.0,
					1.0);

			m_elevator.set(motor_speed);// called 50 times a second
			SmartDashboard.putNumber("Motor Speed", motor_speed);
			SmartDashboard.putString("Using PID", "YES");
			if (m_switch1.isPressed()) // Already returning a true or false value
			{
				m_encoder.setPosition(0);
			}
			// * and a if statement to see if the robot is going up or down(like negative
			// moving motor is going up, positive going down,)
			// * add power, p value when it is going up and else for when it is going down.
			// */
		}
	}
};