// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  // Declare motor controller
  private final WPI_TalonSRX m_armMotor = new WPI_TalonSRX(ArmConstants.ArmLeaderMotorCAN);

  private final static ArmUtils util = new ArmUtils();
  private double m_setpoint = 0;
  private static boolean isLimitSwitchMuted = false;
  private static boolean isHoned = false;
  private static double adjusted_feedforward;

  /** Object of a simulated arm **/
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getCIM(2),
      139.78,
      0.0035, // Moment of intertia
      0.639, // 0.3193m*2
      0,
      Math.PI,
      true,
      0);

  // Intialize SRX simulation
  private final TalonSRXSimCollection m_armMotorSim = new TalonSRXSimCollection(m_armMotor);

  // Create a Mechanism2d visual display for the arm
  private final Mechanism2d m_mech2d = new Mechanism2d(3, 3);
  private final MechanismRoot2d m_armRoot = m_mech2d.getRoot("ArmPivot", 1.5, 0);
  private final MechanismLigament2d m_armPivot = m_armRoot.append(new MechanismLigament2d(
      "Arm",
      0.639,
      Units.radiansToDegrees(armSim.getAngleRads()), // In degrees
      6,
      new Color8Bit(Color.kPurple)));

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    configureMotors();
    configurePID();

    if (RobotBase.isSimulation()) {
      // Add the arm to the Mechanism2d display
      SmartDashboard.putData("Arm", m_mech2d);
    }
  }

  private void configureMotors() {
    // Configure leader motor
    m_armMotor.configFactoryDefault();

    // Configure encoder
    m_armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    // Set velocity and acceleration of motors
    m_armMotor.configMotionCruiseVelocity(200000); // Adjust
    m_armMotor.configMotionAcceleration(500000); // Adjust

    // Configure motor settings
    m_armMotor.configVoltageCompSaturation(12, 0);
    m_armMotor.configPeakCurrentLimit(45);
    m_armMotor.configNeutralDeadband(0.04);

    if (RobotBase.isSimulation()) {
      // Configure motor for simulation
      m_armMotor.setSensorPhase(false);
      m_armMotor.setInverted(true);
    } else {
      // Configure motor for real hardware
      m_armMotor.setSensorPhase(false);
      m_armMotor.setInverted(false);
    }

    // Configure limit switch
    m_armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);
  }

  public void configurePID() {
    /*
     * double ku = 0;
     * double tu = 0;
     * double[] pidConstants = util.setZieglerNicholsConstants(ku, tu);
     */

    // Set PID Constants
    m_armMotor.config_kP(0, ArmConstants.kP);
    m_armMotor.config_kI(0, ArmConstants.kI);
    m_armMotor.config_kD(0, ArmConstants.kD);
  }

  public void setPower(double power) {
    m_armMotor.set(power);
  }

  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
    // Set setpoint in ticks
    m_armMotor.set(ControlMode.MotionMagic, util.degToCTRESensorUnits(setpoint, ArmConstants.EncoderCPR));
  }

  public boolean atSetpoint() {
    return Math.abs(
        util.CTRESensorUnitsToDeg(getAngle(), ArmConstants.EncoderCPR) - m_setpoint) < ArmConstants.PositionTolerance;
  }

  public double getAngle() {
    // Get raw angle of arm in ticks (4096 per revolution)
    return m_armMotor.getSensorCollection().getQuadraturePosition();
  }

  public double modAngleInTicks(double angleInTicks) {
    // Modulo the angle to within 0 - 4096
    return Math.IEEEremainder(angleInTicks, ArmConstants.EncoderCPR);

  }

  public void toggleLimitSwitchMute() {
    isLimitSwitchMuted = !isLimitSwitchMuted;
  }

  public boolean atZeroPos() {
    return m_armMotor.isRevLimitSwitchClosed() == 0; // Switch open
  }

  public void stopMotor() {
    m_armMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      // Reset setpoint to base position
      setSetpoint(ArmConstants.BaseSetpoint);
    }
    if (m_armMotor.isRevLimitSwitchClosed() == 1 && !isLimitSwitchMuted) {
      // Recalibrate arm encoder
      m_armMotor.getSensorCollection().setQuadraturePosition(0, 1);
    }

    // Calculate feedforward
    adjusted_feedforward = ArmConstants.ArmFeedforward
        * Math.cos(util.degToCTRESensorUnits(getAngle(), ArmConstants.EncoderCPR));

    // Update SmartDashboard
    SmartDashboard.putNumber("Arm Angle", util.CTRESensorUnitsToDeg(getAngle(), ArmConstants.EncoderCPR));
    SmartDashboard.putNumber("Encoder ticks", getAngle());
    SmartDashboard.putNumber("Arm Setpoint", m_setpoint);
    SmartDashboard.putNumber("Setpoint Ticks", m_armMotor.getActiveTrajectoryPosition());
    SmartDashboard.putBoolean("Limit switch muted", isLimitSwitchMuted);
    SmartDashboard.putNumber("Arm Feedforward", adjusted_feedforward);
    SmartDashboard.putBoolean("Arm Fwd Limit Switch", m_armMotor.isFwdLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("Arm Rev Limit Switch", m_armMotor.isRevLimitSwitchClosed() == 1);
    SmartDashboard.putNumber("Encoder Output", m_armMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Motor Control Effort", m_armMotor.get());
    SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    SmartDashboard.putNumber("Error", m_armMotor.getClosedLoopError());
    /*
     * SmartDashboard.putNumber("Velocity", m_armLeaderMotor.());
     * SmartDashboard.putNumber("Acceleration", m_armLeaderMotor.);
     */

    if (RobotBase.isSimulation() || !atSetpoint()) {
      // Update the simulation
      m_armMotor.set(
          ControlMode.MotionMagic,
          util.degToCTRESensorUnits(m_setpoint, ArmConstants.EncoderCPR),
          DemandType.ArbitraryFeedForward, // For gravity compensation
          adjusted_feedforward);
    }
  }

  @Override
  public void simulationPeriodic() {
    armSim.update(0.02); // 20ms update time
    armSim.setInput(m_armMotorSim.getMotorOutputLeadVoltage());

    // Update the simulated encoder position
    m_armMotorSim.setQuadratureRawPosition(
        util.radsToCTRESensorUnits(armSim.getAngleRads(), ArmConstants.EncoderCPR));

    // Update the mechanism ligament for visualization
    m_armPivot.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    // Reset encoder position when arm is at rest position
    if (armSim.getAngleRads() == Units.degreesToRadians(ArmConstants.BaseSetpoint)) {
      m_armMotor.getSensorCollection().setQuadraturePosition(0, 0);
    }
    // Update analog position
    m_armMotorSim.setAnalogPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), ArmConstants.EncoderCPR));

  }
}