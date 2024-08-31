// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.teleopCommands.arm.HomeCmd;
import frc.robot.commands.teleopCommands.arm.RaisedSetpoint;
import frc.robot.commands.teleopCommands.drive.DriveFwdCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveFwdAndShoot extends SequentialCommandGroup {
  /** Creates a new ShootNodeCmd. */
  public MoveFwdAndShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(12),
      new DriveFwdCmd().withTimeout(1),
      new ShootNodeCmd() // Shoot and return home
    );
    
  }
}
