package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ExternalFollower;

import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {
        // initialize motors and drivetrain
        public final CANSparkMax m_frontLeftMotor = new CANSparkMax(kFrontLeft,
                        MotorType.kBrushless);
        public final CANSparkMax m_frontRightMotor = new CANSparkMax(kFrontRight,
                        MotorType.kBrushless);
        public final CANSparkMax m_rearLeftMotor = new CANSparkMax(kRearLeft,
                        MotorType.kBrushless);
        public final CANSparkMax m_rearRightMotor = new CANSparkMax(kRearRight,
                        MotorType.kBrushless);
        public final DifferentialDrive m_driveTrain = new DifferentialDrive(m_frontLeftMotor, m_rearRightMotor);

        public DriveSubsystem() {
                // drivetrain
                m_frontLeftMotor.restoreFactoryDefaults();
                m_frontRightMotor.restoreFactoryDefaults();
                m_rearLeftMotor.restoreFactoryDefaults();
                m_rearRightMotor.restoreFactoryDefaults();

                m_frontLeftMotor.setInverted(true);
                m_rearRightMotor.setInverted(false);
                m_rearLeftMotor.follow(m_frontLeftMotor);
                m_frontRightMotor.follow(m_rearRightMotor);
        }

        @Override
        public void periodic() {
                // This method will be called once per scheduler run
                m_driveTrain.tankDrive(
                                RobotContainer.m_driverController.getRawAxis(Constants.ControllerConstants.kLeftVertical),
                                RobotContainer.m_driverController.getRawAxis(Constants.ControllerConstants.kRightVertical));
        }

        // @Override
        // public void simulationPeriodic() {
        // // This method will be called once per scheduler run during simulation
        // }
}
