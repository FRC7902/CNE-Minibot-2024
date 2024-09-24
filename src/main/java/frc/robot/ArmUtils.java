// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class ArmUtils {

    public int radsToCTRESensorUnits(double angleInRads) {
        return (int) (angleInRads * ArmConstants.kEncoderCPR / 2 * Math.PI / ArmConstants.kGearRatio);
    }

    public double CTRESensorUnitsToRads(double angleInSensorUnits) {
        return (angleInSensorUnits * 2 * Math.PI * ArmConstants.kGearRatio / ArmConstants.kEncoderCPR);
    }

    public double degToCTRESensorUnits(double angleInDeg) {
        return (int) (angleInDeg * ArmConstants.kGearRatio * ArmConstants.kEncoderCPR / 360);
    }

    public static double CTRESensorUnitsToDeg(double angleInSensorUnits) {
        return (angleInSensorUnits * 360.0 / ArmConstants.kEncoderCPR / ArmConstants.kGearRatio);
    }

    // ku = ultimate gain, tu = oscillation period
    public double[] setZieglerNicholsConstants(double ku, double tu) {
        double constants[] = new double[3];
        constants[0] = 0.2 * ku;           // kP
        constants[1] = 1.2 * ku / tu;      // kI
        constants[2] = 0.075 * ku * tu;    // kD
        return constants;
      }

    public double encoderVelocityToRPM(double encoderVelocity) {
        return encoderVelocity * 600 / ArmConstants.kEncoderCPR * ArmConstants.kGearRatio;     // Encoder ticks per 100ms
    }

    public double encoderVelocityToDegPerSec(double encoderVelocity) {
        return encoderVelocity * 10 * 360 / ArmConstants.kEncoderCPR * ArmConstants.kGearRatio; 
    }
}