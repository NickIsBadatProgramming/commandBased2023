// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Drive extends CommandBase {

  //velocity values
  double xV, yV, rV;

  //slew rate limiters
  SlewRateLimiter xFilter = new SlewRateLimiter(3);
  SlewRateLimiter yFilter = new SlewRateLimiter(3);
  SlewRateLimiter rFilter = new SlewRateLimiter(0);


  /** Creates a new Drive. */
  public Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(RobotContainer.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    XboxController driveController = RobotContainer.xbox1;
   
    xV = xFilter.calculate(driveController.getRawAxis(0));
    yV = yFilter.calculate(driveController.getRawAxis(1));
    rV = rFilter.calculate(driveController.getRawAxis(4));

    SmartDashboard.putNumber("X velocity", xV);
    SmartDashboard.putNumber("Y Velocity", yV);
    SmartDashboard.putNumber("R Velocity", rV);

    RobotContainer.swerve.Drive(xV, yV, rV);

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
    return false;
  }
}
