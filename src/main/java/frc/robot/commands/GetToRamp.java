// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutonomousConstants;

public class GetToRamp extends CommandBase {

  boolean hasCrossedThreshhold = false;
  AHRS navx;


  /** Creates a new GetToRamp. */
  public GetToRamp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerve);
    navx = RobotContainer.swerve.getNavX();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerve.DriveField(0, -0.25, 0);
    if(navx.getRoll() > AutonomousConstants.rampAngleEntryThreshhold) {
      hasCrossedThreshhold = true;
      RobotContainer.swerve.DriveField(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerve.DriveField(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(navx.getRoll() < AutonomousConstants.rampAngleExitThreshhold && hasCrossedThreshhold) {
      return true;
    }
    return false;
  }
}
