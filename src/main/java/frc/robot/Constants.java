// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveConstants {
        public static final double SWERVE_GEAR_RATIO_DRIVE = 6.75; // Gear ratio of swerve drive
        public static final double SWERVE_GEAR_RATIO_STEER = 6.75;

        public static final double TrackwidthM = 0.596955; //Dimensions for swerve calculation - in meters
        public static final double WheelbaseM = 0.4445;
        public static final double WheelCircumferenceM = 0.315196;
        
        public static final double AdditionalTurnSpeed = 0.28; //additional speed of the motor 180 degrees from the target angle
        public static final double MinModuleTurnSpeed = 0.07;
        public static final double SpeedMultiplier = 0.9; //Forward Speed
        public static final double MinimumSpeedMultiplier = 0.2; //Minimum forward speed multiplier
        public static final double AdditionalSpeed = 1 - MinimumSpeedMultiplier;

        public static final int RefreshRateEncoders = 50;
        public static final int RefreshRateMotors = 50;

        public static final int SlowMode = 2;
        public static final int ErrorMargin = 3;
        public static final int ErrorMarginTurn = 7;


        //Chassis rotational speeds
        public static final double MinChassisTurnSpeed = 0.4;
        public static final double AdditionalChassisTurnSpeed = 0.6;



        //Swerve Zeroes

        public static final double FR_Zero = 353;
        public static final double FL_Zero = 172;
        public static final double BL_Zero = 181;
        public static final double BR_Zero = 219;


        //Autonomous Variables
        public static final double basePathSpeed = 0.15;
        public static final double nearPathSpeed = 0.1;
        public static final double distanceBeforeSlow = 0.1;
        public static final double error = 0.05;

    }
    public static final class ControllerConstants {
        public static final double xboxDeadzone = 0.2;
        public static final double flickStickDeadzone = 0.6;
        public static final double armYDeadzone = 0.3;
    }

    public static final class Config {
        public static final boolean usingFlickStick = false;
        public static final boolean usingLogitech360 = false;
    }
    public static final class AutonomousConstants {
        public static final int AprilTagRedRight = 1; //As viewed from the opposite of the field 
        public static final int AprilTagRedMiddle = 2;
        public static final int AprilTagRedLeft = 3;
        public static final int AprilTagRedCargoCenter = 5;

        public static final int AprilTagBlueRight = 6;
        public static final int AprilTagBlueMiddle = 7;
        public static final int AprilTagBlueLeft = 8;
        public static final int AprilTagBlueCargoCenter = 4;

        public static final String[] IntToApriltags = {"null", "Red Right", "Red Middle", "Red Left", "Blue Cargo", "Red Cargo", "Blue Right", "Blue Middle", "Blue Left"};

        public static final double DistanceBetweenTags = 1.6764; //meters
        public static final double DistanceToCargoCenterTag = 2.32537;
        public static final double CargoCenterTagRecess = 0.665226;

        public static final double limelightXOffset = 0.36;
        public static final double limelightYOffset = 0.15;
        public static final double limelightZOffset = 0.72; //Robot is .14m away from apriltag at this distance, any farther and it will clip out of frame
        



    }
    public static final class ArmConstants {


        public static final double MaxArmLength = 4611; 
        public static final double MinArmLength = 0;
        public static final double ArmLengthError = 10;
        public static final double BaseArmLength = 25.5;

        //pivot constants
        public static final double MaxArmPivotAngle = 160;
        public static final double MinArmPivotAngle = -160;
        public static final double PivotError = 5;
        public static final double PivotOffset = 114.2;

        //Dimensional constraints
        public static final double MaxArmHeight = 0; //All lengths in inches
        public static final double MinArmHeight = -5;
        public static final double MaxFrameExtension = 32;
        public static final double MinArmHeightInsideFrame = -5;
        public static final double FrameAngleBoundsFront = -150;
        public static final double FrameAngleBoundsRear = 150;


        public static final double pivotSpeed = 0.35;
        public static final double winchSpeed = 0.9;

        public static final double ticksPerInch = 0.01002619;

        public static final double slowZone = 0.45; //For the pivot


        //The arm has about 32 inches of extension it can do at level before it goes beyond it's limts or 3191 units
        //The arm can extend up max 0 inches


        //2 + 5/8in = 0
        //11 in = 835.312500

        //8.375in/835.3125units

        //0.01002619in/unit

    }
}
