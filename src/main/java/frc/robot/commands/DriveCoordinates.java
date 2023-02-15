// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public class DriveCoordinates extends CommandBase {

  double x, y, angle;
  double xDifference, yDifference;

  boolean finished = false;

  /** Creates a new DriveCoordinates. */
  public DriveCoordinates(double x, double y, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerve);
    this.x = x;
    this.y = y;
    this.angle = angle;

    System.out.println("Autonomous was called");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed, ySpeed;

    this.xDifference = this.x - RobotContainer.swerve.getOdometryX();
    this.yDifference = this.y - RobotContainer.swerve.getOdometryY();

    xSpeed = (Math.abs(xDifference)/xDifference) * SwerveConstants.basePathSpeed;
    if(xDifference <= SwerveConstants.distanceBeforeSlow) {
      xSpeed = (Math.abs(xDifference)/xDifference) * SwerveConstants.nearPathSpeed;
    }

    ySpeed = (Math.abs(yDifference)/yDifference) * SwerveConstants.basePathSpeed;
    if(yDifference <= SwerveConstants.distanceBeforeSlow) {
      ySpeed = (Math.abs(yDifference)/yDifference) * SwerveConstants.nearPathSpeed;
    }

    SmartDashboard.putNumber("Odometry Y Difference" , this.y);

    if(Math.abs(this.xDifference) < SwerveConstants.error) {
    xSpeed = 0;
    }

    if(Math.abs(this.yDifference) < SwerveConstants.error) {
    ySpeed = 0;
    }

    RobotContainer.swerve.DriveWithAngle(xSpeed, ySpeed, angle);
    RobotContainer.FL.updateMotorSpeeds();
    RobotContainer.FR.updateMotorSpeeds();
    RobotContainer.BL.updateMotorSpeeds();
    RobotContainer.BR.updateMotorSpeeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
