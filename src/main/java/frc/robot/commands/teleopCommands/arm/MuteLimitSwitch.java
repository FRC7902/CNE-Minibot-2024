// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MuteLimitSwitch extends Command {

  /** Creates a new MuteLimitSwitch. */
  public MuteLimitSwitch() {
    addRequirements(RobotContainer.m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_armSubsystem.toggleLimitSwitchMute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
