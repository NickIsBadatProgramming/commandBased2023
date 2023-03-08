// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ResetOdometry extends CommandBase {
  /** Creates a new ResetOdometry. */
  public ResetOdometry() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerve.resetOdometry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.0
  @Override
  public boolean isFinished() {
    return (RobotContainer.swerve.getOdometryX() == 0 && RobotContainer.swerve.getOdometryY() == 0);
  }
}
