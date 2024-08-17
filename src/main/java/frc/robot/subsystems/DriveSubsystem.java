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
  // PID contoller objects
  private final SparkPIDController leftSpeedPID = m_leftLeaderMotor.getPIDController();
  private final SparkPIDController rightSpeedPID = m_rightLeaderMotor.getPIDController();

  // Instantiates the DifferentialDrive class and the XboxController class
  public final DifferentialDrive m_drive;

  // ENCODER DECLARATION
  private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();
  

  private void setPidGains(){
    leftSpeedPID.setP(0.0); //Integral, derivative, and proportional gains for the PID controller
    leftSpeedPID.setD(0.0);
    leftSpeedPID.setI(0.0);
  
    rightSpeedPID.setP(0.0);
    rightSpeedPID.setD(0.0);
    rightSpeedPID.setI(0.0);
  }

  double DriveDistance=0.0; //Allows setDistanceToDrive to accept an argument
  public void setDistanceToDrive(double DriveDistance){

    this.DriveDistance=DriveDistance;
    rightSpeedPID.setReference(DriveDistance,com.revrobotics.CANSparkBase.ControlType.kPosition);
    leftSpeedPID.setReference(DriveDistance,com.revrobotics.CANSparkBase.ControlType.kPosition);  

  }
  
  public void setPoint(){
    //Setpoints for the PID system
    rightSpeedPID.setReference(0,com.revrobotics.CANSparkBase.ControlType.kVelocity);
    leftSpeedPID.setReference(0,com.revrobotics.CANSparkBase.ControlType.kVelocity);
  }


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configurueMotors();

    // Initialize the DifferentialDrive class with the left and right leader motors
    m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);
  }

  private void configurueMotors() {
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

    // This code limits the motor current to 40 amps to prevent overheating

    int rpm=3000;
    int peakCurrent=70;
    int continuousCurrent=40;

    m_leftLeaderMotor.setSmartCurrentLimit(peakCurrent,continuousCurrent,rpm);
    m_rightLeaderMotor.setSmartCurrentLimit(peakCurrent,continuousCurrent,rpm);
    m_leftFollowerMotor.setSmartCurrentLimit(peakCurrent,continuousCurrent,rpm);
    m_rightFollowerMotor.setSmartCurrentLimit(peakCurrent,continuousCurrent,rpm);


  }

  public void curvatureDrive(double moveSpeed, double rotateSpeed) {
    m_drive.curvatureDrive(moveSpeed, rotateSpeed, true);
  }

  
  public void pidControlObjects() {
    //Sets the speed of motors based on the PID control 

    m_leftLeaderMotor.set(leftSpeedPID.getOutputMax());
    m_rightLeaderMotor.set(rightSpeedPID.getOutputMax());
    m_leftFollowerMotor.set(leftSpeedPID.getOutputMax());
    m_rightFollowerMotor.set(rightSpeedPID.getOutputMax());
  }
  //it controls the range of output of the PID controller
  public void pidOutputRange(){
    leftSpeedPID.setOutputRange(0.0,0.0);
    rightSpeedPID.setOutputRange(0.0,0.0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run



  }
}