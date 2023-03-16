// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutonomousConstants;


public class BalanceOnRamp extends CommandBase {

  AHRS navx;
  boolean isFinished = false;
  PIDController kPID;


  /** Creates a new BalanceOnRamp. */
  public BalanceOnRamp() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.swerve);
    navx = RobotContainer.swerve.getNavX();
    kPID = new PIDController(AutonomousConstants.balanceKP, AutonomousConstants.balanceKI, AutonomousConstants.balanceKD);
    kPID.setSetpoint(0);
    RobotContainer.swerve.resetOdometry();
    kPID.setTolerance(AutonomousConstants.balanceTolerance);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveSpeed = -kPID.calculate(navx.getRoll(),0);

    if(driveSpeed >= AutonomousConstants.balanceMaxForwardSpeed) {
      driveSpeed = AutonomousConstants.balanceMaxForwardSpeed;
    }

    if(driveSpeed <= AutonomousConstants.balanceMaxReverseSpeed) {
      driveSpeed = AutonomousConstants.balanceMaxReverseSpeed;
    }

    if(RobotContainer.swerve.getOdometryY() > AutonomousConstants.balanceMaxForwardPosition && driveSpeed > 0) {
      driveSpeed = 0;
    }

    if(RobotContainer.swerve.getOdometryY() < AutonomousConstants.balanceMaxReversePosition && driveSpeed < 0) {
      driveSpeed = 0;
    }

    SmartDashboard.putNumber("Balancer Drive Speed", driveSpeed);
    RobotContainer.swerve.Drive(0, driveSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerve.Drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
