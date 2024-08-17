// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class ArmUtils {

    public int radsToCTRESensorUnits(double angleInRads, int encoderCPR) {
        return (int) (angleInRads * encoderCPR / 2 * Math.PI / ArmConstants.GearRatio);
    }

    public double degToCTRESensorUnits(double angleInDeg, int encoderCPR) {
        return (angleInDeg * ArmConstants.GearRatio * encoderCPR / 360);
    }

    public double CTRESensorUnitsToDeg(double angleInSensorUnits, int encoderCPR) {
        return (angleInSensorUnits * 360.0 / encoderCPR * ArmConstants.GearRatio);
    }

    public double CTRESensorUnitsToRads(double angleInSensorUnits, int encoderCPR) {
        return (angleInSensorUnits * 2 * Math.PI * ArmConstants.GearRatio / encoderCPR);
    }

    // ku = ultimate gain, tu = oscillation period
    public double[] setZieglerNicholsConstants(double ku, double tu) {
        double constants[] = new double[3];
        constants[0] = 0.2 * ku;                 // kP
        constants[1] = 2 * ku / tu;              // kI
        constants[2] = constants[0] * tu / 8;    // kD
        return constants;
      }

}