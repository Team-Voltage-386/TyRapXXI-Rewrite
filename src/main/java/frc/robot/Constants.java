// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

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

    public static final class DriveConstants {
        public static final int kFrontLeft = 4; // CAN (Spark)
        public static final int kFrontRight = 1; // CAN (Spark)
        public static final int kRearLeft = 5; // CAN (Spark)
        public static final int kRearRight = 3; // CAN (Spark)
    }

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

        public static final int kEntranceProximityThreshold = 100;
        public static final int kFeederProximityThreshold = 100;
    }

}
