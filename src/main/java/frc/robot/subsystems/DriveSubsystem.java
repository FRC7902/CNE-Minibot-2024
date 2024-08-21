// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // Left Motors
  private final static CANSparkMax m_leftLeaderMotor = new CANSparkMax(DriveConstants.leftBackCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(DriveConstants.leftFrontCAN,
      CANSparkMax.MotorType.kBrushless);

  // Right Motors
  private final static CANSparkMax m_rightLeaderMotor = new CANSparkMax(DriveConstants.rightFrontCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(DriveConstants.rightBackCAN,
      CANSparkMax.MotorType.kBrushless);

  public DriveSubsystem() {
    // Ensures motors are loaded with the current config, not with a previous
    // config.
    m_leftLeaderMotor.restoreFactoryDefaults();
    m_leftFollowerMotor.restoreFactoryDefaults();
    m_rightLeaderMotor.restoreFactoryDefaults();
    m_rightFollowerMotor.restoreFactoryDefaults();

    // Inverts the leader motor of each side so that the motors aren't going against
    // each other
    m_leftLeaderMotor.setInverted(true);
    m_rightLeaderMotor.setInverted(true);

    // Any updates made to the Leader Motor will additionally be made to the
    // Follower Motor, even without directly updating the Follower Motor
    m_leftFollowerMotor.follow(m_leftLeaderMotor);
    m_rightFollowerMotor.follow(m_rightLeaderMotor);

    m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);

    m_leftLeaderMotor.setSmartCurrentLimit(DriveConstants.peakCurrent, DriveConstants.continuousCurrent,
        DriveConstants.rpm);
    m_rightLeaderMotor.setSmartCurrentLimit(DriveConstants.peakCurrent, DriveConstants.continuousCurrent,
        DriveConstants.rpm);
    m_leftFollowerMotor.setSmartCurrentLimit(DriveConstants.peakCurrent, DriveConstants.continuousCurrent,
        DriveConstants.rpm);
    m_rightFollowerMotor.setSmartCurrentLimit(DriveConstants.peakCurrent, DriveConstants.continuousCurrent,
        DriveConstants.rpm);

    setPIDGains();
  }

  // Allows interfacing with the integrated PID Controller on the motors.
  // PID contoller objects
  private final static SparkPIDController leftSpeedPID = m_leftLeaderMotor.getPIDController();
  private final static SparkPIDController rightSpeedPID = m_rightLeaderMotor.getPIDController();

  // Instantiates the DifferentialDrive class and the XboxController class
  public final DifferentialDrive m_drive;

  // ENCODER DECLARATION
  private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

  private void setPIDGains() {
    leftSpeedPID.setP(0.0); // Integral, derivative, and proportional gains for the PID controller
    leftSpeedPID.setD(0.0);
    leftSpeedPID.setI(0.0);

    rightSpeedPID.setP(0.0);
    rightSpeedPID.setD(0.0);
    rightSpeedPID.setI(0.0);
  }

  // Allows setDistanceToDrive to accept an argument
  public void setDistanceToDrive(double DriveDistance) {

    rightSpeedPID.setReference(DriveDistance, com.revrobotics.CANSparkBase.ControlType.kPosition);
    leftSpeedPID.setReference(DriveDistance, com.revrobotics.CANSparkBase.ControlType.kPosition);

  }

  public static void setVelocity(double velocity) {
    // Setpoints for the PID system

    rightSpeedPID.setReference(velocity, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    leftSpeedPID.setReference(velocity, com.revrobotics.CANSparkBase.ControlType.kVelocity);
  }

  public void curvatureDrive(double moveSpeed, double rotateSpeed) {
    m_drive.curvatureDrive(moveSpeed, rotateSpeed, true);
  }

  // it controls the range of output of the PID controller
  public void pidOutputRange() {
    leftSpeedPID.setOutputRange(0.0, 0.0);
    rightSpeedPID.setOutputRange(0.0, 0.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
