// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CurvatureDriveCommand extends Command {
  /** Creates a new CurvatureDrive. */
  public CurvatureDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(frc.robot.RobotContainer.m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double moveSpeed = ((frc.robot.RobotContainer.m_driverController.getLeftY()) * 0.5);
    double rotateSpeed = ((frc.robot.RobotContainer.m_driverController.getRightX()) * 0.5);

    frc.robot.RobotContainer.m_driveSubsystem.curvatureDrive(rotateSpeed, moveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}