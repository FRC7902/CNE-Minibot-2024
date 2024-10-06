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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ArmConstants;
import frc.robot.ArmUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;

public class ArmSubsystem extends SubsystemBase {
  public final static ArmUtils util = new ArmUtils();
  private double m_setpoint = 0;
  private static boolean isLimitSwitchMuted = false;
  private static boolean isHomed = false;

  // Declare motor controller
  private final WPI_TalonSRX m_armMotor = new WPI_TalonSRX(ArmConstants.kArmMotorCAN);

  // Create a new ArmFeedforward with gains kS, kG, kV, and kA
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
    ArmConstants.kSVolts,
    ArmConstants.kGVolts,
    ArmConstants.kVVoltSecondPerRad,
    ArmConstants.kAVoltSecondSquaredPerRad);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation
  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.zero(Volts);

  // Mutable holder for unit-safe angle values, persisted to avoid reallocation
  private final MutableMeasure<Angle> m_angle = MutableMeasure.zero(Degrees);

  // Mutable holder for unit-safe velocity values, persisted to avoid reallocation
  private final MutableMeasure<Velocity<Angle>> m_velocity = MutableMeasure.zero(DegreesPerSecond);
  
  // Creates a SysIdRoutine for characterizing the arm
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      // Tell SysId how to plumb driving voltage to the motors
      voltage -> m_armMotor.setVoltage(voltage.in(Volts)),

      // Tell SysId how to record a frame of data for the motor on the mechanism being characterized
      log -> {
        // Record a frame for the arm motor
         log.motor("arm-motor")
         .voltage(
            m_appliedVoltage.mut_replace(
            m_armMotor.get() * RobotController.getBatteryVoltage(),   // Get actual voltage of the motor
            Volts))

          .angularPosition(
            m_angle.mut_replace(
            util.CTRESensorUnitsToDeg(getAngle()), 
            Degrees))
              
          .angularVelocity(
            m_velocity.mut_replace(
            util.encoderVelocityToDegPerSec(m_armMotor.getSelectedSensorVelocity()), 
            DegreesPerSecond));
        },
        this, 
        "Arm Subsystem")
);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  /* private final TrapezoidProfile m_profile = 
    new TrapezoidProfile(TrapezoidProfile.Constraints(
      ArmConstants.kVelocitySetpoint,
      ArmConstants.kAccelerationSetpoint
  )); */

  /** Object of a simulated arm **/
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getCIM(1),
      139.78,
      0.0035, // Moment of inertia
      0.639, // 0.3193m*2
      0,
      Math.PI,
      true,
      0);

  // Intialize SRX simulation
  private final TalonSRXSimCollection m_armMotorSim = new TalonSRXSimCollection(m_armMotor);

  // Create a Mechanism2d visual display for the arm
  private final Mechanism2d m_mech2d = new Mechanism2d(80, 80);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_arm =  m_armPivot.append(
    new MechanismLigament2d(
        "Arm",
        40,    
        Units.radiansToDegrees(armSim.getAngleRads()),     
        6, 
        new Color8Bit(Color.kAliceBlue)
    )
  );

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    configureMotors();
    configurePID();

    if (RobotBase.isSimulation()) {
      // Add the arm to the Mechanism2d display
      SmartDashboard.putData("Arm Mech2d", m_mech2d);
    }
  }

  private void configureMotors() {
    // Configure leader motor
    m_armMotor.configFactoryDefault();

    // Configure encoder
    m_armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    // Set velocity and acceleration of motors
    m_armMotor.configMotionCruiseVelocity(50); // Adjust
    m_armMotor.configMotionAcceleration(50); // Adjust

    // Configure motor settings
    m_armMotor.configVoltageCompSaturation(12, 0);
    m_armMotor.configPeakCurrentLimit(70);
    m_armMotor.configContinuousCurrentLimit(45);
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
    /* double ku = 0;
     * double tu = 0;
     * double[] pidConstants = util.setZieglerNicholsConstants(ku, tu);
     */

    // Set PID Constants
    m_armMotor.config_kP(0, ArmConstants.kP);
    m_armMotor.config_kI(0, ArmConstants.kI);
    m_armMotor.config_kD(0, ArmConstants.kD);

  }

  public void setPower(double power) {
    m_armMotor.set(ControlMode.PercentOutput, power);
  }

  public void setSetpoint(double setpoint, int acceleration, int velocity) {
    m_armMotor.configMotionCruiseVelocity(velocity); // Adjust
    m_armMotor.configMotionAcceleration(acceleration); // Adjust
    
    m_setpoint = setpoint;
    m_armMotor.set(ControlMode.MotionMagic, util.degToCTRESensorUnits(setpoint));
  }

  public void setSetpoint(double setpoint) {
    setSetpoint(setpoint, ArmConstants.defaultAcceleration, ArmConstants.defaultSpeed);
  }


  public void setSetpoint(double setpoint, ControlMode mode) {
    m_armMotor.set(mode, util.degToCTRESensorUnits(setpoint));
  };

  public void softLimit(double setpoint) {
    m_armMotor.configForwardSoftLimitThreshold(util.degToCTRESensorUnits(ArmConstants.kMaxAngle));
    m_armMotor.configForwardSoftLimitEnable(true);
  }

  public boolean atSetpoint() {
    return Math.abs(
        util.CTRESensorUnitsToDeg(getAngle()) - m_setpoint) < ArmConstants.kPositionTolerance;
  }

  public double getAngle() {
    // Get raw angle of arm in ticks (4096 per revolution)
    return m_armMotor.getSensorCollection().getQuadraturePosition();
  }

  public double modAngleInTicks(double angleInTicks) {
    // Modulo the angle to within 0 - 4096
    return Math.IEEEremainder(angleInTicks, ArmConstants.kEncoderCPR);
  }

  public void toggleLimitSwitchMute() {
    isLimitSwitchMuted = !isLimitSwitchMuted;
  }

  public boolean isLimitSwitchPressed() {
    return isLimitSwitchMuted;
  };

  public boolean atZeroPos() {
    return m_armMotor.isRevLimitSwitchClosed() == 0; // Switch open
  }

  public void stopMotor() {
    m_armMotor.stopMotor();
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (DriverStation.isDisabled()) {
      // Reset setpoint to base position
      setSetpoint(ArmConstants.kBaseSetpoint);
    }
    if (m_armMotor.isRevLimitSwitchClosed() == 1 && !isLimitSwitchMuted) {
      // Recalibrate arm encoder
      m_armMotor.getSensorCollection().setQuadraturePosition(0, 1);
    }

    if (atZeroPos() && m_setpoint == 0) {
      m_armMotor.stopMotor();
    }

    // Calculate feedforward, assume accel is 0
    double feedforward = m_feedforward.calculate(
      util.CTRESensorUnitsToRads(getAngle()),
      ArmConstants.kVelocitySetpoint,
      ArmConstants.kAccelerationSetpoint
    );

    // Calculate feedforward at current angle
    double adjusted_feedforward = feedforward * Math.cos(util.degToCTRESensorUnits(getAngle()));

    if (RobotBase.isSimulation() || !atSetpoint()) {
      m_armMotor.set(
          ControlMode.MotionMagic,
          util.degToCTRESensorUnits(m_setpoint),
          DemandType.ArbitraryFeedForward, // For gravity compensation
          adjusted_feedforward);
    }
    
    // Update SmartDashboard
    SmartDashboard.putNumber("Arm Angle", util.CTRESensorUnitsToDeg(getAngle()));
    SmartDashboard.putNumber("Encoder ticks", getAngle());
    SmartDashboard.putNumber("Arm Setpoint", m_setpoint);
    SmartDashboard.putNumber("Setpoint Ticks", m_armMotor.getActiveTrajectoryPosition());
    SmartDashboard.putBoolean("Limit switch muted", isLimitSwitchMuted);
    SmartDashboard.putNumber("Arm Feedforward", adjusted_feedforward);
    SmartDashboard.putNumber("Encoder Output", m_armMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Motor Control Effort", m_armMotor.get());
    SmartDashboard.putBoolean("At Setpoint", atSetpoint());
    SmartDashboard.putNumber("Error", m_armMotor.getClosedLoopError());
    SmartDashboard.putNumber("Arm Velocity (deg/s)", util.encoderVelocityToDegPerSec(m_armMotor.getSelectedSensorVelocity()));
    SmartDashboard.putBoolean("Arm Homed", isHomed);
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(m_armMotorSim.getMotorOutputLeadVoltage());
    armSim.update(0.02); 

    // Update the simulated encoder position
    m_armMotorSim.setQuadratureRawPosition(
        util.radsToCTRESensorUnits(armSim.getAngleRads()));

    // Update the mechanism ligament for visualization
    m_arm.setAngle(Math.toDegrees(armSim.getAngleRads()));

    // Reset encoder position when arm is at rest position
    if (armSim.getAngleRads() == Units.degreesToRadians(ArmConstants.kBaseSetpoint)) {
      m_armMotor.getSensorCollection().setQuadraturePosition(0, 0);
    }

    // Update analog position
    m_armMotorSim.setAnalogPosition(util.radsToCTRESensorUnits(armSim.getAngleRads()));

  }
}