// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GetArmToPoint extends CommandBase {

  double angle;
  double extension;
  boolean isFinished;

  /** Creates a new GetArmToPoint. */
  public GetArmToPoint(double angle, double extension) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.grabber);
    this.angle = angle;
    this.extension = extension;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isFinished = RobotContainer.grabber.getToPosition(angle, extension);
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
