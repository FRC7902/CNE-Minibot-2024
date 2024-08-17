// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class BaseSetpoint extends Command {
  private final ArmSubsystem m_armSubsystem;

  /** Creates a new BaseSetpointCmd. */
  public BaseSetpoint(ArmSubsystem arm) {
    m_armSubsystem =  arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.setSetpoint(ArmConstants.BaseSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.atSetpoint();
  }
}
