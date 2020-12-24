/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int gyroPort = 0;
    

    public static class DrivetrainPorts{
        public static final int leftMasterPort = 0;
        public static final int leftFollowerPort = 0;
        public static final int rightMasterPort = 0;
        public static final int rightFollowerPort = 0;
    };

    public static class DrivetrainNumericalConstants{
        public static final double kMaxSpeed = 3.0; // meters per second->needs to be inputted
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
        public static final double kTrackWidth = 0.381 * 2; // meters->needs to be inputted
        public static final double kWheelRadius = 0.0762; // meters->needs to be inputted
        public static final int kEncoderResolution = 2048; // <-precision for integrated talon FX encoder
        public static final double distancePerPulseFactor = (2 * Math.PI * kWheelRadius)/ kEncoderResolution;
    };

    public static class PIDConstants{
        public static final double gyrokP = 0.005;
        public static final double drivekP = 1;
        public static final double driveKI = 1;
        public static final double driveKD = 1;

        public static final double ksetPoint = 0.0;
    };
}
