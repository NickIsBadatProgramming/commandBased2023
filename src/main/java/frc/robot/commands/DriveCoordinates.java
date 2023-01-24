// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants;

public class DriveCoordinates extends CommandBase {

  double x, y, angle;

  boolean finished = false;

  /** Creates a new DriveCoordinates. */
  public DriveCoordinates(double x, double y, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerve);
    this.x = x;
    this.y = y;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.resetFeild.schedule();
    RobotContainer.resetOdometry.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed, ySpeed;

    if(Math.abs(this.x - RobotContainer.swerve.getOdometry()[0]) < SwerveConstants.distanceBeforeSlow) {
      if(Math.abs(this.x - RobotContainer.swerve.getOdometry()[0]) < SwerveConstants.error) {
        xSpeed = 0;
      } else {
      xSpeed = SwerveConstants.nearPathSpeed;
      }
    } else {
      xSpeed = SwerveConstants.basePathSpeed;
    }

    if(Math.abs(this.y - RobotContainer.swerve.getOdometry()[0]) < SwerveConstants.distanceBeforeSlow) {
      if(Math.abs(this.y - RobotContainer.swerve.getOdometry()[0]) < SwerveConstants.error) {
        ySpeed = 0;
      } else {
      ySpeed = SwerveConstants.nearPathSpeed;
      }
    } else {
      ySpeed = SwerveConstants.basePathSpeed;
    }

    if((this.x/Math.abs(this.x)) < 0) {
      xSpeed *= -1;
    }

    if((this.y/Math.abs(this.y)) < 0) {
      ySpeed *= -1;
    }

    RobotContainer.swerve.DriveWithAngle(xSpeed, ySpeed, angle);

    if(ySpeed == 0 && xSpeed == 0) {
      this.finished = true;
    }


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
