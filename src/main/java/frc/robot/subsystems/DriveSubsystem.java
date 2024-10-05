// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // Left Motors
  private final CANSparkMax m_leftLeaderMotor = new CANSparkMax(DriveConstants.kLeftBackCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_leftFollowerMotor = new CANSparkMax(DriveConstants.kLeftFrontCAN,
      CANSparkMax.MotorType.kBrushless);

  // Right Motors
  private final CANSparkMax m_rightLeaderMotor = new CANSparkMax(DriveConstants.kRightBackCAN,
      CANSparkMax.MotorType.kBrushless);
  private final CANSparkMax m_rightFollowerMotor = new CANSparkMax(DriveConstants.kRightFrontCAN,
      CANSparkMax.MotorType.kBrushless);

  // Allows interfacing with the integrated PID Controller on the motors.
  private final SparkPIDController m_leftPID = m_leftLeaderMotor.getPIDController();
  private final SparkPIDController m_rightPID = m_rightLeaderMotor.getPIDController();

  // Instantiates the DifferentialDrive class and the XboxController class
  public final DifferentialDrive m_drive;

  // ENCODER DECLARATION
  private final RelativeEncoder m_leftEncoder = m_leftLeaderMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderMotor.getEncoder();

  // Dummy encoder declaration
  private final Encoder m_leftDummyEncoder = new Encoder(12, 13);
  private final Encoder m_rightDummyEncoder = new Encoder(14, 15);
  private final AnalogGyro m_DummyGyro = new AnalogGyro(1);

  private final EncoderSim m_rightEncoderSim;
  private final EncoderSim m_leftEncoderSim;
  private final AnalogGyroSim m_gyroSim;
  private final DifferentialDrivetrainSim m_driveTrainSim;
  private final Field2d m_Field2d;

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configureMotors();

    // Initialize the DifferentialDrive class with the left and right leader motors
    m_drive = new DifferentialDrive(m_leftLeaderMotor, m_rightLeaderMotor);
    m_drive.setDeadband(0.05);

    m_leftDummyEncoder.setDistancePerPulse(0.1524 * Math.PI / 1024);
    m_rightDummyEncoder.setDistancePerPulse(0.1524 * Math.PI / 1024);

    m_leftEncoderSim = new EncoderSim(m_leftDummyEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightDummyEncoder);
    m_gyroSim = new AnalogGyroSim(m_DummyGyro);

    // Define physical parameters of simulated drivetrain
    m_driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide,
        KitbotGearing.k10p71,
        KitbotWheelSize.kSixInch,
        null);

    m_Field2d = new Field2d();
    SmartDashboard.putData(m_Field2d);

    if (Robot.isSimulation()) {
      m_odometry = new DifferentialDriveOdometry(
          m_DummyGyro.getRotation2d(),
          m_leftDummyEncoder.getDistance(),
          m_rightDummyEncoder.getDistance(),
          new Pose2d(1, 1, new Rotation2d()));
    } else {
      m_odometry = new DifferentialDriveOdometry(
          m_DummyGyro.getRotation2d(),
          m_leftEncoder.getPosition(),
          m_rightEncoder.getPosition(),
          new Pose2d(1, 1, new Rotation2d()));
    }

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

    m_rightLeaderMotor.setIdleMode(IdleMode.kBrake);
    m_rightFollowerMotor.setIdleMode(IdleMode.kBrake);
    m_leftLeaderMotor.setIdleMode(IdleMode.kBrake);
    m_leftFollowerMotor.setIdleMode(IdleMode.kBrake);

  }

  public void resetEncoders() {
    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);
  }

  public void stop() {
    m_rightLeaderMotor.set(0);
    m_leftLeaderMotor.set(0);
  }

  public void setPowerRight(double power) {
    m_rightLeaderMotor.set(power);
  }

  public void setPowerLeft(double power) {
    m_leftLeaderMotor.set(power);
  }

  public void curvatureDrive(double moveSpeed, double rotateSpeed) {
    m_drive.curvatureDrive(moveSpeed, rotateSpeed, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Right drive", m_rightLeaderMotor.get());
    SmartDashboard.putNumber("Left drive", m_leftLeaderMotor.get());

    m_odometry.update(
        m_DummyGyro.getRotation2d(),
        m_leftDummyEncoder.getDistance(),
        m_rightDummyEncoder.getDistance());

    m_Field2d.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {

    m_driveTrainSim.setInputs(
        m_leftLeaderMotor.get() * RobotController.getInputVoltage(),
        -m_rightLeaderMotor.get() * RobotController.getInputVoltage());

    m_driveTrainSim.update(0.02);

    // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveTrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveTrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveTrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveTrainSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(m_driveTrainSim.getHeading().getDegrees());
    
  }

}