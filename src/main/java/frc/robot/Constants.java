// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**Button number mappings*/
    public static final class ControllerConstants {

        public static final int kLeftVertical = 1;
        public static final int kRightVertical = 5;
        public static final int kLeftHorizontal = 0;
        public static final int kRightHorizontal = 4;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;

        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftOptions = 7;
        public static final int kRightOptions = 8;
        public static final int kLeftJoystickPressed = 9;
        public static final int kRightJoystickPressed = 10;

    }

    /**Drive motor CAN bus addresses*/
    public static final class DriveConstants {
        public static final int kFrontLeft = 4; // CAN (Spark)
        public static final int kFrontRight = 1; // CAN (Spark)
        public static final int kRearLeft = 5; // CAN (Spark)
        public static final int kRearRight = 3; // CAN (Spark)
        public static final double kMPR = 0.4788;
        public static final TalonSRX kGyro = new TalonSRX(BallMovementConstants.kFeeder);
    }

    /**PID paramaters*/
    public static final class pidConstants {
        // LimeLight arcade drive pid constants:
        public static final double LLP = 0.04; // P
        public static final double LLI = 0.025;  // I
        public static final double LLD = 0.001; // D
        public static final double LLC = 0.5; // Clamp

        public static final double LLPB = 0.04; // P
        public static final double LLIB = 0.001;  // I
        public static final double LLDB = 0.0005; // D

        public static final double HP = 10; // P
        public static final double HI = 0.5;  // I
        public static final double HD = 0; // D
        public static final double HC = 1;

        public static final double LP = 0.00045;
        public static final double LI = 0.0003;
        public static final double LD = 0.00002;
    }

    /**Constants for the limelight used for estimating distance*/
    public static final class LimeLightConstants {
        public static final double camHeightHoop = 0.63;
        public static final double camEleAngleHoop = 12.5;
        public static final double targetHeightHoop = 2.6416;
        public static final double camHeightBall = 0.72;
        public static final double camEleAngleBall = -13;
        public static final double targetHeightBall = 0.12065;
        public static final double targetLostWaitTime = 0.1;
    }

    /**Constants part of the BallMovementSubsystem*/
    public static final class BallMovementConstants {
        public static final int kPneumaticsModule = 0; // PCM
        public static final int kBallPickupForwardChannel = 0; // PCM
        public static final int kBallPickupReverseChannel = 1; // PCM
        public static final DoubleSolenoid.Value kIntakeDeployed = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value kIntakeRetracted = DoubleSolenoid.Value.kReverse;

        public static final int kLauncherLead = 9; // CAN (Spark)
        public static final int kLauncherFollower = 8; // CAN (Spark)
        public static final int kFeeder = 9; // CAN (Talon)
        public static final int kSerializerMotor = 3; // CAN (Talon)
        public static final int kIntakeMotor = 7; // CAN (Talon)
        public static final int kIndexer = 9; // DIO
        public static final int kEntrance = 7; // DIO
        public static final int kFeederSensor = 10; // DIO (MoreBoard Slot 0)

        public static final ColorSensorV3 entranceSensor = new ColorSensorV3(I2C.Port.kOnboard);
        public static final ColorSensorV3 feederSensor = new ColorSensorV3(I2C.Port.kMXP);

        public static final int kEntranceProximityThreshold = 100;
        public static final int kFeederProximityThreshold = 100;

        public static final double serializerPower = 0.8;
        public static final double intakePower = 1;
        public static final double feederPower = 0.6;
        public static final int launcherSpeedSet = 2800;
        public static final int launcherSpeedTolerances = 100;
        public static final double manHoodSpeed = -0.01;

        // ADD 10 FOR DIO ON MORE BOARD
        public static final int kHoodMotor = 6; // CAN (Talon)
        public static final int kHoodEncoder = 12; // PWM plugged into DIO (More Board slot 2)
        public static final int kHoodLimit = 23; // DIO (More Board slot 13)
        public static final int kTurretMotor = 4; // CAN (Talon)
        public static final int kTurretEncoder = 21; // DIO
        public static final int kTurretLimit = 22; // DIO (MoreBoard slot 12)
        public static final TalonSRX hoodMotor = new TalonSRX(kHoodMotor);
        public static final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(kHoodEncoder);
        public static final DigitalInput hoodLimit = new DigitalInput(kHoodLimit);
    }
}