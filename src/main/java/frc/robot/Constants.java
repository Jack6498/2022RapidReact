// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {

        public static final int leftLeaderCANId = 1;
        public static final int rightLeaderCANId = 2;
        public static final int leftFollowerCANId = 3;
        public static final int rightFollowerCANId = 4;
        public static final Pose2d red1StartingPose = 
            new Pose2d(
                new Translation2d(8.33, 6.18), 
                new Rotation2d()
            );
        public static final Pose2d red2StartingPose = 
            new Pose2d(
                new Translation2d(9.28, 5.76), 
                new Rotation2d()
            );
        public static final Pose2d red3StartingPose = 
            new Pose2d(
                new Translation2d(10.29, 6.19), 
                new Rotation2d()
            );
        public static final Pose2d blue1StartingPose = 
            new Pose2d(
                new Translation2d(6.27, 4.88), 
                new Rotation2d()
            );
        public static final Pose2d blue2StartingPose = 
            new Pose2d(
                new Translation2d(7.00, 2.65), 
                new Rotation2d()
            );
        public static final Pose2d blue3StartingPose = 
            new Pose2d(
                new Translation2d(8.05, 2.113), 
                new Rotation2d()
            );
        public static final double wheelDiameterInches = 4;
        public static final double gearRatio = 30;
        public static final double trackWidthInches = 36;
        public static final double robotMassPounds = 130;
        public static final double robotMOI = 4;

    }

    public static final class ShooterConstants {
        public static final int flywheelCANId = 14;
        public static final double flywheel_kP = 0.0001;
    }
}
