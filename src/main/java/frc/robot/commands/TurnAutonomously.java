// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;


import java.lang.Math;

public class TurnAutonomously extends Command {
  /** Creates a new CurvatureDrive. */
  public TurnAutonomously(double angleOfRotation) {
    DriveSubsystem driveSubsystem = new DriveSubsystem();
    final double distanceToTurn = ((RobotConstants.trackWidth) / Math.sin(angleOfRotation));
    final double originalEncoderValue = driveSubsystem.m_leftEncoder.getPosition();

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
    double rotateSpeed = 1;

    RobotContainer.m_driveSubsystem.curvatureDrive(rotateSpeed, 0);
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
