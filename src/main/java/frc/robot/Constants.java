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

    //Motor IDs, currently not the correct IDs, will change later.
    public static final int leftBackCAN = 30;
    public static final int leftFrontCAN = 31;
    public static final int rightBackCAN = 32;
    public static final int rightFrontCAN = 33;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}

