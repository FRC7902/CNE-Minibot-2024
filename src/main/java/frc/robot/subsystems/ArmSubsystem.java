// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.ArmUtils;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final static ArmUtils util = new ArmUtils();
  
  // Declare motor controllers
  private final WPI_TalonSRX m_armLeaderMotor = new WPI_TalonSRX(ArmConstants.ArmLeaderMotorCAN);
  private final WPI_VictorSPX m_armPivotFollower =  new WPI_VictorSPX(ArmConstants.ArmFollowerMotorCAN);
  
  /** Object of a simulated arm */
  /*private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
  //    DCMotor gearbox - 2
  //    double gearing
  //    double jKgMetersSquared
  //    double armLengthMeters
  //    double minAngleRads
  //    double maxAngleRads
  //    boolean simulateGravity - true
  //    double startingAngleRads
  );*/

  /* 
  private final Mechanism2d m_mech2d;
  private final MechanismLigament2d m_armLigament;
  private final TalonSRXSimCollection m_armLeaderMotorSim; */

  private double m_setpoint = 0;

  private static boolean isLimitSwitchMuted = false;

  //private final DriveSubsystem m_driveSubsystem;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {    
    //m_driveSubsystem = drive;    

    configureMotors();
  }

  private void configureMotors() {
    // Configure leader motor
    m_armLeaderMotor.configFactoryDefault();
    m_armLeaderMotor.setSensorPhase(true);
    m_armLeaderMotor.setInverted(true);
    m_armLeaderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    // Set velocity and acceleration of motors
    m_armLeaderMotor.configMotionCruiseVelocity(200);
    m_armLeaderMotor.configMotionAcceleration(500);

    // Configure follower motor
    m_armPivotFollower.configFactoryDefault();
    m_armPivotFollower.follow(m_armLeaderMotor);
    m_armPivotFollower.setInverted(InvertType.FollowMaster);

    m_armLeaderMotor.configVoltageCompSaturation(12,0);  // 12V
    m_armLeaderMotor.configPeakCurrentLimit(45);                      // 45A 
    m_armLeaderMotor.configNeutralDeadband(0.04);          // 4% deadband

    // Configure limit switch
    m_armLeaderMotor.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, 
        LimitSwitchNormal.NormallyOpen
    );

    // Tuning PID Constants
    double ku = 0;
    double tu = 0;
    double[] pidConstants = util.setZieglerNicholsConstants(ku, tu);

    // Set PID Constants
    m_armLeaderMotor.config_kP(0, pidConstants[0]);
    m_armLeaderMotor.config_kI(0, pidConstants[1]);
    m_armLeaderMotor.config_kD(0, pidConstants[2]);
  }

  public void setPower(double power) {
    m_armLeaderMotor.set(power);
  }

  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
    // Set setpoint in ticks
    m_armLeaderMotor.set(ControlMode.Position, util.degToCTRESensorUnits(setpoint, ArmConstants.EncoderCPR));
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle() - m_setpoint) < ArmConstants.PositionTolerance;
  }

  public void usePIDOutput() {
    // Use PID output to set motor power
    m_armLeaderMotor.set(ControlMode.Position, m_setpoint);
  }

  public double getAngle() {
    // Get angle of arm in ticks (4096 per revolution) 
    return modAngleInTicks(
        m_armLeaderMotor.getSensorCollection().getQuadraturePosition() / ArmConstants.EncoderToOutputRatio
    );
  }

  public double modAngleInTicks(double angleInTicks) {
    // Modulo the angle to within 0 - 4096
    return Math.IEEEremainder(angleInTicks, ArmConstants.EncoderCPR);
  }

  public void toggleLimitSwitchMute() {
    isLimitSwitchMuted = !isLimitSwitchMuted;
    System.out.println("Limit switch muted: " + isLimitSwitchMuted);
  }

  public boolean atZeroPos() {
    return m_armLeaderMotor.isFwdLimitSwitchClosed() == 0; // Switch open
  }

  public void stopMotor() {
    m_armLeaderMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update SmartDashboard
    SmartDashboard.putNumber("Arm Angle: ", getAngle());
    SmartDashboard.putNumber("Arm Setpoint: ", m_setpoint);

    SmartDashboard.putBoolean("Limit switch muted: ", isLimitSwitchMuted);
 
  }

  @Override
  public void simulationPeriodic() {
    // Add simulation code here if needed

  }
}