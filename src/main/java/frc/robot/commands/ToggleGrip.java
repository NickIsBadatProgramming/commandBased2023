// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ToggleGrip extends CommandBase {
  /** Creates a new ToggleGrip. */
  public ToggleGrip() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.grabber);
  }

  boolean isFinished = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.grip = !RobotContainer.grip;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.grip) {
      RobotContainer.grabber.grabberSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      RobotContainer.grabber.grabberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    if(RobotContainer.grip && RobotContainer.grabberSolenoid.get() == DoubleSolenoid.Value.kForward ) {
      isFinished = true;
    }

    if(!RobotContainer.grip && RobotContainer.grabberSolenoid.get() == DoubleSolenoid.Value.kReverse ) {
      isFinished = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
