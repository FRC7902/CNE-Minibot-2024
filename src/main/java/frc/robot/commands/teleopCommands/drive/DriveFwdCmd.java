// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

//drive backwards for auton
public class DriveFwdCmd extends Command {

  /** Creates a new AutoDriveFwdCmd. */
  public DriveFwdCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_driveSubsystem.setPowerRight(-0.5);
    RobotContainer.m_driveSubsystem.setPowerLeft(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
