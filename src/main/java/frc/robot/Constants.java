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
        public static final float MODULE_TURN_TIME_SECONDS = 1; //how long it takes for the modules to turn the correct amount
        public static final int MOTOR_MAX_RPM = 3000;

        public static final double WheelbaseM = 0.596955; //Dimensions for swerve calculation - in meters
        public static final double TrackwidthM = 0.596955;
        public static final double WheelCircumferenceM = 0.1;

        public static final double FalconSpeedConstant = 2048/60000; //Converting m/s to steps/millisecond
    }
}
