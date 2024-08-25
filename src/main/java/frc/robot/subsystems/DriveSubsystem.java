// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;


public class DriveSubsystem extends SubsystemBase {

  // Left Motors
  private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(DriveConstants.leftBackCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(DriveConstants.leftFrontCAN,
      CANSparkMax.MotorType.kBrushless);

  // Right Motors
  private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(DriveConstants.rightFrontCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(DriveConstants.rightBackCAN,
      CANSparkMax.MotorType.kBrushless);

  // Allows interfacing with the integrated PID Controller on the motors.
  private final SparkPIDController m_leftPID = m_leftLeaderMotor.getPIDController();
  private final SparkPIDController m_rightPID = m_rightLeaderMotor.getPIDController();

  // Instantiates the DifferentialDrive class and the XboxController class
  public final DifferentialDrive m_drive;

  // ENCODER DECLARATION
  public final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  public final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

  public double rightEncoderValue, leftEncoderValue;

  private final DifferentialDriveKinematics m_differentialDriveKinematics = new DifferentialDriveKinematics(RobotConstants.trackWidth);


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configureMotors();

    // Initialize the DifferentialDrive class with the left and right leader motors
    m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);
    m_leftEncoder.setPositionConversionFactor((RobotConstants.wheelDiameter * Math.PI) / RobotConstants.gearRatio);
    m_rightEncoder.setPositionConversionFactor((RobotConstants.wheelDiameter * Math.PI) / RobotConstants.gearRatio);    

    m_leftEncoder.setVelocityConversionFactor((RobotConstants.wheelDiameter * Math.PI) / (RobotConstants.gearRatio * 60));
    m_rightEncoder.setVelocityConversionFactor((RobotConstants.wheelDiameter * Math.PI) / (RobotConstants.gearRatio * 60));

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

  }

  private void configureMotors() {
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

    // Sets the current limit of the motors to 45 amps to prevent overheating
    m_leftLeaderMotor.setSmartCurrentLimit(45);
    m_rightLeaderMotor.setSmartCurrentLimit(45);
    m_leftFollowerMotor.setSmartCurrentLimit(45);
    m_rightFollowerMotor.setSmartCurrentLimit(45);
  }

  public void curvatureDrive(double moveSpeed, double rotateSpeed) {
    m_drive.curvatureDrive(moveSpeed, rotateSpeed, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightEncoderValue = m_rightEncoder.getPosition();
    leftEncoderValue = m_leftEncoder.getPosition();    
    SmartDashboard.putNumber("right encoders", rightEncoderValue);
    SmartDashboard.putNumber("left encoders", leftEncoderValue);
    
  }
}