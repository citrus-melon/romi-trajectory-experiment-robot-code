// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double COUNTS_PER_REVOLUTION = 1440;
    public static final double WHEEL_DIAMETER_METER = 70d / 1000d;
    public static final double KS = 0.085302;
    public static final double KV = 11.119;
    public static final double KA = 1.9463;
    public static final double KP = 14.877;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(141d/1000d);
    public static final double MAX_VELOCITY = 4;
    public static final double MAX_ACCELERATION = 3;
}
