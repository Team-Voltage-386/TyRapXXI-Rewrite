package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class DriveSubsystem extends SubsystemBase {

        // initialize motors and drivetrain
        private final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeft,MotorType.kBrushless);
        private final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRight,MotorType.kBrushless);
        private final CANSparkMax rearLeftMotor = new CANSparkMax(Constants.DriveConstants.kRearLeft,MotorType.kBrushless);
        private final CANSparkMax rearRightMotor = new CANSparkMax(Constants.DriveConstants.kRearRight,MotorType.kBrushless);
        private final DifferentialDrive driveTrain = new DifferentialDrive(rearLeftMotor, frontRightMotor);

        PigeonIMU _pigeon;
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        double[] ypr = new double[3];
        Pose2d pos = new Pose2d();
        DifferentialDriveOdometry odometry;

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
                odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getRawHeading()));
                _pigeon = new PigeonIMU(kGyro);
                resetOdometery(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));
        }

        @Override
        public void periodic() {
                updateIMU();
                updateOdometery();
        }

        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        public void resetOdometery(Pose2d pose) {
                resetEncoders();
                odometry.resetPosition(pose, Rotation2d.fromDegrees(getRawHeading()));
        }

        // arcade drive method to be called by commands
        public void arcadeDrive(Double forwardPower, Double turnPower) {
                driveTrain.arcadeDrive(forwardPower*kSpeedLimit, turnPower*kSpeedLimit);
        }

        //tank drive method to be called by commands
        public void tankDrive(Double leftPower, Double rightPower) {
                driveTrain.tankDrive(leftPower*kSpeedLimit, rightPower*kSpeedLimit);
        }

        public void resetEncoders() {
                rightEncoder.setPosition(0.0);
                leftEncoder.setPosition(0.0);
        }

        public double getRawHeading() {
                double y = -ypr[0];
                while (y < 0) y += 360;
                while (y > 360) y -= 360;
                return y;
        }

        public double getHeadingError(double sp) {
                double v = sp-getPose().getRotation().getDegrees();
                while (v < -180) v += 360;
                while (v > 180) v -= 360;
                return v;
        }

        private void updateIMU() {
                _pigeon.getGeneralStatus(genStatus);
                _pigeon.getYawPitchRoll(ypr);
        }

        private void updateOdometery() {
                odometry.update(Rotation2d.fromDegrees(getRawHeading()), leftEncoder.getPosition(),rightEncoder.getPosition());
        }
}