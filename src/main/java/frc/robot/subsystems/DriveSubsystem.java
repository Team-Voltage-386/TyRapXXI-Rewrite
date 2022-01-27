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

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ExternalFollower;

import static frc.robot.Constants.DriveConstants.*;

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
        public RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
        public RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
        
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

        }

        @Override
        public void periodic() {
                 //This method will be called once per scheduler run
                /*
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
                 rightEncoderWidget.setDouble(rightEncoder.getPosition());*/

        }

        // arcade drive method to be called by commands
        public void arcadeDrive(Double forwardPower, Double turnPower) {
                driveTrain.arcadeDrive(forwardPower, turnPower);
        }

        //tank drive method to be called by commands
        public void tankDrive(Double leftPower, Double rightPower) {
                driveTrain.tankDrive(leftPower, rightPower);
        }

        @Override
        public void simulationPeriodic() {
                // This method will be called once per scheduler run during simulation
        }
}
