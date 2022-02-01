package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static frc.robot.Constants.DriveConstants.*;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class DriveSubsystem extends SubsystemBase {

        // initialize motors and drivetrain
        private final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeft,MotorType.kBrushless);
        private final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRight,MotorType.kBrushless);
        private final CANSparkMax rearLeftMotor = new CANSparkMax(Constants.DriveConstants.kRearLeft,MotorType.kBrushless);
        private final CANSparkMax rearRightMotor = new CANSparkMax(Constants.DriveConstants.kRearRight,MotorType.kBrushless);
        private final DifferentialDrive driveTrain = new DifferentialDrive(rearLeftMotor, frontRightMotor);

        // Sensor instantiations
        RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
        RelativeEncoder rightEncoder = frontRightMotor.getEncoder();


        /**Creates a new DriveSubsystem*/
        public DriveSubsystem() {
                rearLeftMotor.setInverted(true);
                frontRightMotor.setInverted(false);
                frontLeftMotor.follow(rearLeftMotor);// front left yields faulty encoder values so that set follower
                rearRightMotor.follow(frontRightMotor);
                leftEncoder.setPositionConversionFactor(kMPR);
                rightEncoder.setPositionConversionFactor(kMPR);
        }

        public void linearDrive(double meters) {
                
        }

        // arcade drive method to be called by commands
        public void arcadeDrive(Double forwardPower, Double turnPower) {
                driveTrain.arcadeDrive(forwardPower, turnPower);
        }

        //tank drive method to be called by commands
        public void tankDrive(Double leftPower, Double rightPower) {
                driveTrain.tankDrive(leftPower, rightPower);
        }

        public void resetEncoders() {
                rightEncoder.setPosition(0.0);
                leftEncoder.setPosition(0.0);
        }
}
