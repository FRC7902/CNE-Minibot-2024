// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

public class MoveArmDownCmd extends Command {


  /** Creates a new MoveArmDownCmd. */
  public MoveArmDownCmd() {
    addRequirements(RobotContainer.m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_armSubsystem.atZeroPos()) {
      RobotContainer.m_armSubsystem.setSetpoint(0, ArmConstants.defaultAcceleration, ArmConstants.defaultSpeed);
    } else {
      RobotContainer.m_armSubsystem.stopMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_armSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
