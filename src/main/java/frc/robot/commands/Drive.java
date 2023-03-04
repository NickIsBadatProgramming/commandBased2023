// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Config;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;

public class Drive extends CommandBase {

  //velocity values
  double xV, yV, rV, fX, fY, rV2, xV2, yV2, speedMultiplier;

  //slew rate limiters
  SlewRateLimiter xFilter = new SlewRateLimiter(0.8);
  SlewRateLimiter yFilter = new SlewRateLimiter(0.8);
  SlewRateLimiter rFilter = new SlewRateLimiter(4);




  /** Creates a new Drive. */
  public Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(RobotContainer.swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    

   


    SmartDashboard.putBoolean("Is using field", RobotContainer.isUsingField);


    if(Config.usingLogitech360) {
      Joystick driveController = RobotContainer.logitech3d;





      //xV = -driveController.getRawAxis(0);
      xV = -xFilter.calculate(driveController.getRawAxis(0));
      if (Math.abs(xV) < ControllerConstants.xboxDeadzone) xV = 0;
      //yV = -driveController.getRawAxis(1);
      yV = -yFilter.calculate(driveController.getRawAxis(1));
      if (Math.abs(yV) < ControllerConstants.xboxDeadzone) yV = 0;
      rV = -driveController.getRawAxis(2);
      //rV = -rFilter.calculate(driveController.getRawAxis(2));
      if (Math.abs(rV) < ControllerConstants.xboxDeadzone) rV = 0;

      speedMultiplier = SwerveConstants.MinimumSpeedMultiplier + (((-driveController.getRawAxis(3) + 1)/2) * SwerveConstants.AdditionalSpeed);

      xV *= speedMultiplier;
      yV *= speedMultiplier;
      rV *= speedMultiplier;

    } else {

      XboxController driveController = RobotContainer.xbox1;
      if (Math.abs(driveController.getRawAxis(0)) < ControllerConstants.xboxDeadzone){
         xV = -xFilter.calculate(0);
      } else xV = -xFilter.calculate(driveController.getRawAxis(0));

      if (Math.abs(driveController.getRawAxis(1)) < ControllerConstants.xboxDeadzone) {
        yV = -yFilter.calculate(0);
      } else yV = -yFilter.calculate(driveController.getRawAxis(1));

      if (Math.abs(driveController.getRawAxis(4)) < ControllerConstants.xboxDeadzone) {
        rV = -rFilter.calculate(0);
      } else rV = -rFilter.calculate(driveController.getRawAxis(4));

      if(RobotContainer.rightBumper.getAsBoolean()) {
        xV /= SwerveConstants.SlowMode;
        yV /= SwerveConstants.SlowMode;
      }

      //flick stick stuff
      fX = driveController.getRawAxis(4);
      if (Math.abs(fX) < ControllerConstants.flickStickDeadzone) fX = 0;
      fY = -driveController.getRawAxis(5);
      if (Math.abs(fY) < ControllerConstants.flickStickDeadzone) fY = 0;

    }

    XboxController secondaryController = RobotContainer.xbox2;
    rV2 = (secondaryController.getLeftTriggerAxis() - secondaryController.getRightTriggerAxis())/2;
    if(Math.abs(rV2) < ControllerConstants.xboxDeadzone) {
      rV2 = 0;
    }

    xV2 = ((-secondaryController.getRawAxis(0)));
    if (Math.abs(xV2) < ControllerConstants.flickStickDeadzone) xV2 = 0;

    yV2 = ((-secondaryController.getRawAxis(1)));
    if (Math.abs(yV2) < ControllerConstants.flickStickDeadzone) yV2 = 0;

    boolean usingSecondary = false;

    if((xV == 0 && yV == 0 && rV == 0) && (xV2 != 0 || yV2 != 0 || rV2 != 0)) {
      xV = xV2/3;
      yV = yV2/3;
      rV = rV2;
      usingSecondary = true;
    }



    

    SmartDashboard.putNumber("X velocity", xV);
    SmartDashboard.putNumber("Y Velocity", yV);
    SmartDashboard.putNumber("R Velocity", rV);

    if(RobotContainer.isUsingField && !usingSecondary) {
      if(Config.usingFlickStick) {
        RobotContainer.swerve.DriveWithAngle(xV, yV, fX, fY);
      }
      else {
        RobotContainer.swerve.DriveField(xV, yV, rV);
      }
    } else {
        RobotContainer.swerve.Drive(xV, yV, rV);
    }

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
