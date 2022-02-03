package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix.sensors.PigeonIMU;
// import com.ctre.phoenix.sensors.PigeonIMU.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ExternalFollower;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutonomousConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import java.lang.Math;

public class DriveSubsystem extends SubsystemBase {

        // initialize motors and drivetrain
        public final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeft,
                        MotorType.kBrushless);
        public final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRight,
                        MotorType.kBrushless);
        public final CANSparkMax rearLeftMotor = new CANSparkMax(Constants.DriveConstants.kRearLeft,
                        MotorType.kBrushless);
        public final CANSparkMax rearRightMotor = new CANSparkMax(Constants.DriveConstants.kRearRight,
                        MotorType.kBrushless);
        public final DifferentialDrive driveTrain = new DifferentialDrive(rearLeftMotor, frontRightMotor);

        // Sensor instantiations
        RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
        RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
        PigeonIMU _pigeon;
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        double[] ypr = new double[3];

        // Odometry class for tracking robot pose
        private final DifferentialDriveOdometry odometry;

        // Creates a shuffleboard tab for the drive
        private ShuffleboardTab tab = Shuffleboard.getTab("Drive");

        // Create output widgets
        private NetworkTableEntry frontLeftOutputWidget = tab.add("F-L Output", 0).withPosition(0, 0).getEntry();
        private NetworkTableEntry frontRightOutputWidget = tab.add("F-R Output", 0).withPosition(1, 0).getEntry();
        private NetworkTableEntry backLeftOutputWidget = tab.add("B-L Output", 0).withPosition(0, 1).getEntry();
        private NetworkTableEntry backRightOutputWidget = tab.add("B-R Output", 0).withPosition(1, 1).getEntry();

        // Create temperature widgets
        private NetworkTableEntry frontLeftTempWidget = tab.add("F-L Temp", 0).withPosition(3, 0).getEntry();
        private NetworkTableEntry frontRightTempWidget = tab.add("F-R Temp", 0).withPosition(4, 0).getEntry();
        private NetworkTableEntry backLeftTempWidget = tab.add("B-L Temp", 0).withPosition(3, 1).getEntry();
        private NetworkTableEntry backRightTempWidget = tab.add("B-R Temp", 0).withPosition(4, 1).getEntry();

        // Create current widgets
        private NetworkTableEntry frontLeftCurrentWidget = tab.add("F-L Current", 0).withPosition(6, 0).getEntry();
        private NetworkTableEntry frontRightCurrentWidget = tab.add("F-R Current", 0).withPosition(7, 0).getEntry();
        private NetworkTableEntry backLeftCurrentWidget = tab.add("B-L Current", 0).withPosition(6, 1).getEntry();
        private NetworkTableEntry backRightCurrentWidget = tab.add("B-R Current", 0).withPosition(7, 1).getEntry();

        // Create encoder widgets
        private NetworkTableEntry leftEncoderWidget = tab.add("Left Encoder", 0).withSize(2, 1).withPosition(2, 2)
                        .getEntry();
        private NetworkTableEntry rightEncoderWidget = tab.add("Right Encoder", 0).withSize(2, 1).withPosition(4, 2)
                        .getEntry();

        public DriveSubsystem() {
                // drivetrain
                frontLeftMotor.restoreFactoryDefaults();
                frontRightMotor.restoreFactoryDefaults();
                rearLeftMotor.restoreFactoryDefaults();
                rearRightMotor.restoreFactoryDefaults();

                rearLeftMotor.setInverted(true);
                frontRightMotor.setInverted(false);
                frontLeftMotor.follow(rearLeftMotor);// front left yields faulty encoder values so that set follower
                rearRightMotor.follow(frontRightMotor);

                _pigeon = new PigeonIMU(kGyro);
                odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
                // rotations to meters
                leftEncoder.setPositionConversionFactor(kPositionFactor);
                rightEncoder.setPositionConversionFactor(kPositionFactor);

                resetEncoders();

        }

        @Override
        public void periodic() {
                // This method will be called once per scheduler run
                _pigeon.getGeneralStatus(genStatus);

                _pigeon.getYawPitchRoll(ypr);
                System.out.println("Yaw:" + getYaw());
                System.out.println("Heading:" + getHeading());

                // Update output widgets
                frontLeftOutputWidget.setDouble(frontLeftMotor.get());
                frontRightOutputWidget.setDouble(frontRightMotor.get());
                backLeftOutputWidget.setDouble(rearLeftMotor.get());
                backRightOutputWidget.setDouble(rearRightMotor.get());

                // Update temp widgets
                frontLeftTempWidget.setDouble(frontLeftMotor.getMotorTemperature());
                frontRightTempWidget.setDouble(frontRightMotor.getMotorTemperature());
                backLeftTempWidget.setDouble(rearLeftMotor.getMotorTemperature());
                backRightTempWidget.setDouble(rearRightMotor.getMotorTemperature());

                // Update current widgets
                frontLeftCurrentWidget.setDouble(frontLeftMotor.getOutputCurrent());
                frontRightCurrentWidget.setDouble(frontRightMotor.getOutputCurrent());
                backLeftCurrentWidget.setDouble(rearLeftMotor.getOutputCurrent());
                backRightCurrentWidget.setDouble(rearRightMotor.getOutputCurrent());

                // Update encoder widgets
                leftEncoderWidget.setDouble(leftEncoder.getPosition());
                rightEncoderWidget.setDouble(rightEncoder.getPosition());

                odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getPosition(),
                                rightEncoder.getPosition());

        }

        // arcade drive method to be called by commands
        public void arcadeDrive(Double forwardPower, Double turnPower) {
                driveTrain.arcadeDrive(forwardPower, turnPower);
        }

        // tank drive method to be called by commands
        public void tankDrive(Double leftPower, Double rightPower) {
                driveTrain.tankDrive(leftPower, rightPower);
        }

        public double getYaw() {
                return ypr[0];
        }

        public double getHeading() {
                return Math.IEEEremainder(getYaw(), 360.0);
        }

        public void resetEncoders() {
                leftEncoder.setPosition(0.0);
                rightEncoder.setPosition(0.0);
        }

        /**
         * Returns the currently-estimated pose of the robot.
         *
         * @return The pose.
         */
        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        /**
         * Resets the odometry to the specified pose.
         *
         * @param pose The pose to which to set the odometry.
         */
        public void resetOdometry(Pose2d pose) {
                resetEncoders();
                odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
        }

        /**
         * Returns the current wheel speeds of the robot.
         *
         * @return The current wheel speeds.
         */
        public DifferentialDriveWheelSpeeds getDifferentialDriveWheelSpeeds() {
                return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
        }

        /**
         * Controls the left and right sides of the drive directly with voltages.
         *
         * @param leftVolts  the commanded left output
         * @param rightVolts the commanded right output
         */
        public void tankDriveVolts(double leftVolts, double rightVolts) {
                frontLeftMotor.setVoltage(leftVolts);
                frontRightMotor.setVoltage(rightVolts);
                driveTrain.feed();
        }

        /**
         * Gets the average distance of the two encoders.
         *
         * @return the average of the two encoder readings
         */
        public double getAverageEncoderDistance() {
                return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
        }

        /**
         * Gets the left drive encoder.
         *
         * @return the left drive encoder
         */
        public RelativeEncoder getLeftEncoder() {
                return leftEncoder;
        }

        /**
         * Gets the right drive encoder.
         *
         * @return the right drive encoder
         */
        public RelativeEncoder getRightEncoder() {
                return rightEncoder;
        }

        /**
         * Sets the max output of the drive. Useful for scaling the drive to drive more
         * slowly.
         *
         * @param maxOutput the maximum output to which the drive will be constrained
         */
        public void setMaxOutput(double maxOutput) {
                driveTrain.setMaxOutput(maxOutput);
        }

        /** Zeroes the heading of the robot. */
        public void zeroHeading() {
                _pigeon.setYaw(0.0);
        }

        @Override
        public void simulationPeriodic() {
                // This method will be called once per scheduler run during simulation
        }
}
