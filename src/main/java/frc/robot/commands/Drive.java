// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  public Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveFR.set(ControlMode.PercentOutput, 1d);
    RobotContainer.driveFL.set(ControlMode.PercentOutput, -1d);
    RobotContainer.driveBL.set(ControlMode.PercentOutput, -1d);
    RobotContainer.driveBR.set(ControlMode.PercentOutput, 1d);

    RobotContainer.steerFR.set(ControlMode.PercentOutput, 0.05d);
    RobotContainer.steerFL.set(ControlMode.PercentOutput, 0.05d);
    RobotContainer.steerBL.set(ControlMode.PercentOutput, 0.05d);
    RobotContainer.steerBR.set(ControlMode.PercentOutput, 0.05d);


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
