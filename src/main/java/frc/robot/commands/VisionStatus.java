// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class VisionStatus extends CommandBase {
  /** Creates a new VisionStatus. */
  public VisionStatus() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Vision command executed");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putBoolean("Target Detected", RobotContainer.vision.isTarget());

    SmartDashboard.putNumber("X Translation", RobotContainer.vision.tx());
    SmartDashboard.putNumber("Y Translation", RobotContainer.vision.ty());
    SmartDashboard.putNumber("Z Translation", RobotContainer.vision.tz());
    SmartDashboard.putNumber("Target Tag", RobotContainer.vision.target());
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
