package frc.robot;

import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
        public static final class ElevatorConfig {
                public static final SparkMaxConfig m_elavator = new SparkMaxConfig();
                static {
                        m_elavator
                                        .idleMode(IdleMode.kBrake)
                                        .openLoopRampRate(0.30)
                                        .smartCurrentLimit(30, 80);
                }
        }

        public static final class CorallatorConfig {
                public static final LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();
                public static final SparkMaxConfig m_corallatorConfig = new SparkMaxConfig();
                static {
                        limitSwitchConfig
                                        .forwardLimitSwitchEnabled(true)
                                        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
                        m_corallatorConfig
                                        .apply(limitSwitchConfig)
                                        .idleMode(IdleMode.kCoast)
                                        .openLoopRampRate(0.05)
                                        .smartCurrentLimit(80, 60);
                }
                public static final SparkMaxConfig m_coralAngleator = new SparkMaxConfig();
                static {
                        m_coralAngleator
                                        .idleMode(IdleMode.kCoast)
                                        .openLoopRampRate(0.05)
                                        .smartCurrentLimit(10, 20);
                }
        }

        public static final class ClimberConfig {
                public static final SparkMaxConfig m_climberConfig = new SparkMaxConfig();
                static {
                        m_climberConfig
                                .smartCurrentLimit(60);
                }
        }

        public static final class MAXSwerveModule {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward gain.
                        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                                        / ModuleConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;
                        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .openLoopRampRate(0.10)
                                        .smartCurrentLimit(20, 80);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor) // meters
                                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(0.04, 0, 0)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(-1, 1);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(10, 40);
                        turningConfig.absoluteEncoder
                                        // Invert the turning encoder, since the output shaft rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .inverted(true)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(1, 0, 0)
                                        .outputRange(-1, 1)
                                        // Enable PID wrap around for the turning motor. This will allow the PID
                                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                                        // to 10 degrees will go through 0 rather than the other direction which is a
                                        // longer route.
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
