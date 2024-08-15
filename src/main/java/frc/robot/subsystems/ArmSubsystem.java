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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  
  // Declare motor controllers
  private final WPI_TalonSRX m_armLeaderMotor;
  private final WPI_VictorSPX m_armPivotFollower;
  
  private final DriveSubsystem m_driveSubsystem;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(DriveSubsystem drive) {
    m_driveSubsystem = drive;
    
    // Initialize motor controllers
    m_armLeaderMotor = new WPI_TalonSRX(ArmConstants.ArmLeaderMotorCAN);
    m_armPivotFollower = new WPI_VictorSPX(ArmConstants.ArmFollowerMotorCAN);
    
    // Configure motors
    configureMotors();
  }

  private void configureMotors() {
    // Configure leader motor
    m_armLeaderMotor.configFactoryDefault();
    m_armLeaderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    m_armLeaderMotor.setSensorPhase(true);
    m_armLeaderMotor.setInverted(true);

    // Configure follower motor
    m_armPivotFollower.configFactoryDefault();
    m_armPivotFollower.follow(m_armLeaderMotor);
    m_armPivotFollower.setInverted(InvertType.FollowMaster);

    // Configure limit switch
    m_armLeaderMotor.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, 
        LimitSwitchNormal.NormallyOpen
    );
  }

  public void setPower(double power) {
    m_armLeaderMotor.set(power);
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

  public boolean atZeroPos() {
    return m_armLeaderMotor.isFwdLimitSwitchClosed() == 0; // Switch open
  }

  public void stopMotor() {
    m_armLeaderMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // Add simulation code here if needed
  }
}