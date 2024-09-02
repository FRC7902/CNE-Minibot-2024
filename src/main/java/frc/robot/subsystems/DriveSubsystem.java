// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SparkPIDController m_leftPID = m_leftLeaderMotor.getPIDController();
  private final SparkPIDController m_rightPID = m_rightLeaderMotor.getPIDController();

  
  // Instantiates the DifferentialDrive class and the XboxController class
  public final DifferentialDrive m_drive;

  // ENCODER DECLARATION
  private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

  public void invertRightmotor(){
    m_rightLeaderMotor.setInverted(true);
  }


 // public void stopreverseRotation(){
   // if m_leftEncoder.getVelocity(); 
//this code will stop the motor from going into the other direction 
  //}


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configureMotors();
    configurePID();
    pidOutputRange();

    // Initialize the DifferentialDrive class with the left and right leader motors
    m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);
  }

  private void configureMotors() {
    // Ensures motors are loaded with the current config, not with a previous
    // config.
    m_leftLeaderMotor.restoreFactoryDefaults();
    m_leftFollowerMotor.restoreFactoryDefaults();
    m_rightLeaderMotor.restoreFactoryDefaults();
    m_rightFollowerMotor.restoreFactoryDefaults();
    m_leftLeaderMotor.enableVoltageCompensation(12);
    m_rightLeaderMotor.enableVoltageCompensation(12);
    

    // Inverts the leader motor of each side so that the motors aren't going against
    // each other
    m_leftLeaderMotor.setInverted(false);
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

  private void configurePID() {
    m_leftPID.setP(0.000349); // Integral, derivative, and proportional gains for the PID controller

    m_leftPID.setD(0.000001);




    m_leftPID.setI(0.0000005);

    m_leftPID.setFF(0.00005);

    m_rightPID.setP(0.000349);

    m_rightPID.setD(0.000001);
    m_rightPID.setI(0.0000005);

    m_rightPID.setFF(0.00005);

    
  }

  // Allows setDistanceToDrive to accept an argument
  public void setDistanceToDrive(double DriveDistance) {
    m_rightPID.setReference(DriveDistance, com.revrobotics.CANSparkBase.ControlType.kPosition);
    m_leftPID.setReference(DriveDistance, com.revrobotics.CANSparkBase.ControlType.kPosition);
  }

  // Setpoints for the PID system
  public void setVelocity(double velocity) {
    m_rightPID.setReference(velocity, ControlType.kVelocity);
    m_leftPID.setReference(velocity, ControlType.kVelocity);
  }

  public void curvatureDrive(double moveSpeed, double rotateSpeed) {
    m_drive.curvatureDrive(moveSpeed, rotateSpeed, true);
  }

  // it controls the range of output of the PID controller
  public void pidOutputRange() {
    m_leftPID.setOutputRange(0.0, 1.0);
    m_rightPID.setOutputRange(0.0, 1.0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Encoder 1", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Encoder 2", m_rightEncoder.getVelocity());

  


    SmartDashboard.updateValues();
    SmartDashboard.updateValues();



  }
  //Create drivetrain simulation object😘👌

  private DifferentialDrivetrainSim driveSimulate=new DifferentialDrivetrainSim(DCMotor.getNeo550(2),7.29,7.5,60.0,0.0762,0.7112,VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  
  //Encoder simulation🎵📏📐
  private EncoderSim simulateLeftEncoder=new EncoderSim(null);
  private EncoderSim simulateRightEncoder=new EncoderSim(null);


  //Drivetrain simulation method🤯💻
  @Override
  public void simulationPeriodic(){ 
  // Set inputs to the simulated drivetrain, like the motor voltage 
    driveSimulate.setInputs(0.0,0.0);
  // Make the simulated drivetrain update
    driveSimulate.update(0.00);
  // Update all of our sensors, which are encoders in this case. I'm guessing its a feedback loop or something like that. 
    simulateLeftEncoder.setDistance(0.0);
    simulateRightEncoder.setRate(0.0);
    simulateLeftEncoder.setDistance(0.0);
    simulateRightEncoder.setRate(0.0);
  } 
} 


//https://codedocs.revrobotics.com/java/com/revrobotics/revphysicssim we should simulate in this file
//https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
//https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/updating-drivetrain-model.html
//https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/package-summary.html