// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ZeroArm extends CommandBase {
  WaitCommand wait;
  /** Creates a new ZeroArm. */
  public ZeroArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wait = new WaitCommand(2);
    RobotContainer.winchCAN.setPosition(0);
    RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 0);
    wait.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wait.isFinished();
  }
}
