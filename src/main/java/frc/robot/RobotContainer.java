// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autonomousCommands.MoveFwdAndShoot;
import frc.robot.commands.autonomousCommands.ShootNodeCmd;
import frc.robot.commands.teleopCommands.arm.BaseSetpoint;
import frc.robot.commands.teleopCommands.arm.RaisedSetpoint;
import frc.robot.commands.teleopCommands.arm.MoveArmDownCmd;
import frc.robot.commands.teleopCommands.arm.MoveArmUpCmd;
import frc.robot.commands.teleopCommands.arm.MuteLimitSwitch;
import frc.robot.commands.teleopCommands.drive.CurvatureDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  // Controllers
  public static final CommandPS5Controller m_driverController = new CommandPS5Controller(
      OperatorConstants.kDriverControllerPort);
  public static final CommandPS5Controller m_operatorController = new CommandPS5Controller(
      OperatorConstants.kOperatorControllerPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    m_driveSubsystem.setDefaultCommand(new CurvatureDriveCommand());

    // Arm-related commands
    m_operatorController.triangle().whileTrue(new MoveArmUpCmd());
    m_operatorController.cross().whileTrue(new MoveArmDownCmd());
    m_operatorController.circle().onTrue(new BaseSetpoint());
    //m_operatorController.square().onTrue(new ShootNodeCmd());
    m_operatorController.square().onTrue(new RaisedSetpoint());
    // Mute limit switch when the Right D-pad is held
    m_operatorController.povRight().onTrue(new MuteLimitSwitch());

    // SysId bindings for arm characterization
    // Using the left bumper as a modifier for SysId commands so that we can have both
    // sets of bindings as once.
    m_operatorController.L1().and(m_operatorController.triangle())  // Quasistatic forward test
        .onTrue(m_armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_operatorController.L1().and(m_operatorController.cross())    // Quasistatic reverse test
        .onTrue(m_armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    
    m_operatorController.L1().and(m_operatorController.circle())  // Dyanmic forward test
        .onTrue(m_armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_operatorController.L1().and(m_operatorController.square())  // Dynamic reverse test
        .onTrue(m_armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));  
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new MoveFwdAndShoot();
  }
}
