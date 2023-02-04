// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;

public class Grab extends CommandBase {

  double pivot, extend;
  /** Creates a new Grabber. */
  public Grab() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pivot = -RobotContainer.xbox2.getRawAxis(1);
    if(Math.abs(pivot) < ControllerConstants.xboxDeadzone) {
      pivot = 0;
    }
    extend = RobotContainer.xbox2.getRawAxis(5);
    if(Math.abs(extend) < ControllerConstants.xboxDeadzone) {
      extend = 0;
    }


    RobotContainer.grabber.grab(RobotContainer.grip);

    RobotContainer.grabber.pivot(pivot);
    RobotContainer.grabber.winch(extend);

    SmartDashboard.putBoolean("grab", RobotContainer.grip);
    SmartDashboard.putNumber("Pivot", pivot);
    SmartDashboard.putNumber("Winch", extend);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
