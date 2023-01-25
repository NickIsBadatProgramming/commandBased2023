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
        public static final double SpeedMultiplier = 0.4; //Forward Speed

        public static final int RefreshRateEncoders = 50;
        public static final int RefreshRateMotors = 50;

        public static final int SlowMode = 2;
        public static final int ErrorMargin = 3;
        public static final int ErrorMarginTurn = 7;


        //Chassis rotational speeds
        public static final double MinChassisTurnSpeed = 0.4;
        public static final double AdditionalChassisTurnSpeed = 0.6;



        //Swerve Zeroes

        public static final double FR_Zero = 187.2;
        public static final double FL_Zero = 210.4;
        public static final double BL_Zero = 352.4;
        public static final double BR_Zero = 164.1;


        //Autonomous Variables
        public static final double basePathSpeed = 0.3;
        public static final double nearPathSpeed = 0.15;
        public static final double distanceBeforeSlow = 0.1;
        public static final double error = 0.05;

    }
    public static final class ControllerConstants {
        public static final double xboxDeadzone = 0.2;
        public static final double flickStickDeadzone = 0.6;
    }

    public static final class Config {
        public static final boolean usingFlickStick = false;
        public static final boolean usingLogitech360 = true;
    }
}
