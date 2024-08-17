// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MuteLimitSwitch extends Command {
  private final ArmSubsystem m_armSubsystem;

  /** Creates a new MuteLimitSwitch. */
  public MuteLimitSwitch(ArmSubsystem arm) {
    m_armSubsystem = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.toggleLimitSwitchMute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
