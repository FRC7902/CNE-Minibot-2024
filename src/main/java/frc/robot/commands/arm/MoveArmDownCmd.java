// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmDownCmd extends Command {
  private ArmSubsystem m_armSubsystem;
  private static final double ARM_SPEED = -0.5;

  /** Creates a new MoveArmDownCmd. */
  public MoveArmDownCmd(ArmSubsystem arm) {
    m_armSubsystem = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_armSubsystem.atZeroPos()) {
      m_armSubsystem.setPower(ARM_SPEED);
    } else {
      m_armSubsystem.stopMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end() {
    m_armSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
