// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class DriveConstants {

    // Multiplier for the speed of the robot
    public static final double kDriveSpeedMultiplier = 0.75;

    // Motor Controller CAN IDs
    public static final int kLeftBackCAN = 38;
    public static final int kLeftFrontCAN = 39;
    public static final int kRightBackCAN = 36;
    public static final int kRightFrontCAN = 37;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class ArmConstants {
    // CAN IDs for motor
    public static final int kArmMotorCAN = 22;

    // Arm speed
    public static final double kArmSpeed = 0.75;

    // Encoder constants
    public static final int kEncoderCPR = 4096;
    public static final double kEncoderToOutputRatio = 3;

    // PID gains
    public static final double kP = 15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // https://www.reca.lc/arm?armMass=%7B%22s%22%3A6%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A12.57%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A45%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A35%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22CIM%22%7D&ratio=%7B%22magnitude%22%3A100%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
    public static final double kG = 0.42;
    public static final double kS = 0;  // Obtain from sysld
    public static final double kV = 2.15;  // Unused in position control
    // public static final double kA = 0.0; // Omitted 

    public static final double kVelocitySetpoint = 4.89; 
    public static final double kAccelerationSetpoint = 39.1;

    // Setpoints in degrees
    public static final double kBaseSetpoint = 0;
    public static final double kRaisedSetpoint = 35;
    public static final double kPositionTolerance = 5;

    public static final double kGearRatio = 3; 
    public static final double kMaxAngle = 70.0;
    public static final double kArmFeedForward = 2.22 / 12; // T = 8.52N*m

    public static final int defaultSpeed = 400;
    public static final int defaultAcceleration = 400;

  }

  public static class AutoConstants {

  }
}