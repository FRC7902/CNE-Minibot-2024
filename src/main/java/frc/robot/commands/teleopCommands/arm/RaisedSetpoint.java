// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.ArmUtils;

public class RaisedSetpoint extends Command {

  private ArmUtils util = new ArmUtils();

  /** Creates a new RaisedSetpointCmd. */
  public RaisedSetpoint() {
    addRequirements(RobotContainer.m_armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_armSubsystem.setPower(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_armSubsystem.setSetpoint(ArmConstants.kRaisedSetpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return util.CTRESensorUnitsToDeg(RobotContainer.m_armSubsystem.getAngle()) > ArmConstants.kRaisedSetpoint;
  }
}