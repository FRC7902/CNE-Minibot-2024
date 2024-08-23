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
  public static class DriveConstants {

    // Multiplier for the speed of the robot
    public static final double driveSpeedMultiplier = 0.5;

    // Motor Controller CAN IDs
    public static final int leftBackCAN = 32;
    public static final int leftFrontCAN = 31;
    public static final int rightBackCAN = 33;
    public static final int rightFrontCAN = 34;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class RobotConstants {
    // Measurement of robot and certain components of it (in centimeters)
    public static final double trackWidth = 1;
    public static final double wheelCircumference = 47.88;
  }
}

