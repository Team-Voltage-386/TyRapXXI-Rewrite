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
import static frc.robot.Constants.ControllerConstants.*;

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

        protected double slowSpeedFactor = 0.6;
        protected double topSpeedFactor = 1;
        protected double safetySpeedFactor = 0.5;
        protected double driveSpeedFactor;
        protected double turnSpeedFactor;

        protected int direction = -1;// -1 is turret is front, 1 is turret is back

        protected boolean topGear = false;
        protected boolean safetyMode = false; // edit for safety mode

        @Override
        public void periodic() {
                // This method will be called once per scheduler run
                topGear = (RobotContainer.m_driverController.getRawAxis(kRightTrigger) >= 0.1);
                if (topGear) {
                        driveSpeedFactor = topSpeedFactor;
                } else {
                        driveSpeedFactor = slowSpeedFactor;
                }
                turnSpeedFactor = slowSpeedFactor;
                // turnSpeedFactor = driveSpeedFactor;
                if (safetyMode) {
                        driveSpeedFactor = driveSpeedFactor * safetySpeedFactor;
                        turnSpeedFactor = turnSpeedFactor * safetySpeedFactor;
                }
                m_driveTrain.arcadeDrive(
                                driveSpeedFactor * direction
                                                * RobotContainer.m_driverController.getRawAxis(kLeftVertical),
                                turnSpeedFactor * -1
                                                * RobotContainer.m_driverController.getRawAxis(kRightHorizontal));
        }

        // @Override
        // public void simulationPeriodic() {
        // // This method will be called once per scheduler run during simulation
        // }
}
