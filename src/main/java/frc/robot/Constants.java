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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0; 
  }

  public static class ArmConstants {
    // CAN IDs for motor
    public static final int ArmLeaderMotorCAN = 22;

    // Encoder constants
    public static final int EncoderCPR = 4096;
    public static final double EncoderToOutputRatio = 3;

    // PID gains
    public static final double kP = 15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Setpoints in degrees
    public static final double BaseSetpoint = 0;
    public static final double RaisedSetpoint = 90;
    public static final double PositionTolerance = 5;   

    public static final double GearRatio = 3;  
    public static final double MaxAngle = 90;    
    public static final double ArmFeedforward = 0.88 / 12;  // T = 8.52N*m
  }
}