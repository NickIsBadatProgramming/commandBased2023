// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//POSITIONAL DEFINITIONS

//0 Degrees = grabber is facing forwards parallel to the robot frame
//0 Extension = grabber is at minimum extension 


package frc.robot.subsystems;

import java.util.Random;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

public class Grabber extends SubsystemBase {

  public static TalonFX winchMotor, pivotMotor;
  public static PneumaticHub pneumaticHub;
  public static DoubleSolenoid grabberSolenoid;
  public static DoubleSolenoid brakeSolenoid;

  public static CANCoder winchCoder, pivotCoder;
  public static RollingAverage winchHeight, winchExtension;

  public static boolean isBrake = false;

  
  /** Creates a new Grabber. */
  public Grabber() {

    
    
    winchMotor = RobotContainer.winchMotor;
    pivotMotor = RobotContainer.pivotMotor;
    pneumaticHub = RobotContainer.pneumaticHub;
    grabberSolenoid = RobotContainer.grabberSolenoid;
    brakeSolenoid = RobotContainer.brakeSolenoid;

    winchCoder = RobotContainer.winchCAN;
    pivotCoder = RobotContainer.pivotCAN;
    winchHeight = new RollingAverage(8);
    winchExtension = new RollingAverage(8);
  }
 
  public void pivot (double speed) { //This should be -1 or 1
    double position = getCurrentAngle();
    //speed = -speed;

    if(RobotContainer.enableLimits) {

      if(speed == 0) {
        pivotMotor.set(ControlMode.PercentOutput, 0);
        brakeSolenoid.set(DoubleSolenoid.Value.kReverse); //Normally kForward
      } else {
        brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
      }

      if(speed < 0) {
        if(position < ArmConstants.MinArmPivotAngle + ArmConstants.PivotError) {
          pivotMotor.set(ControlMode.PercentOutput, 0);
        } else {
          pivotMotor.set(ControlMode.PercentOutput, speed * ArmConstants.pivotSpeed);
        }
      } 

      if(speed > 0) {
        if(position > ArmConstants.MaxArmPivotAngle - ArmConstants.PivotError) {
          pivotMotor.set(ControlMode.PercentOutput, 0);
        } else {
          pivotMotor.set(ControlMode.PercentOutput, speed * ArmConstants.pivotSpeed);
        }
      }




    } else {
      pivotMotor.set(ControlMode.PercentOutput, speed * ArmConstants.winchSpeed);
    }
  }

  public void winch (double speed) {

    speed = -speed;
    double position = winchCoder.getPosition(); //double position = winchMotor.getSelectedSensorPosition() - RobotContainer.winchZero;

    if(RobotContainer.enableLimits) {

      if(speed < 0) {
        if(position < ArmConstants.MinArmLength + ArmConstants.ArmLengthError) {
          winchMotor.set(ControlMode.PercentOutput, 0);
          RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 1);
        } else {
          winchMotor.set(ControlMode.PercentOutput, speed * ArmConstants.winchSpeed);
          RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 0);
        }
      } 

      if(speed > 0) {
        if(position > ArmConstants.MaxArmLength - ArmConstants.ArmLengthError) {
          winchMotor.set(ControlMode.PercentOutput, 0);
          RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 1);
        } else {
          winchMotor.set(ControlMode.PercentOutput, speed * ArmConstants.winchSpeed);
          RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 0);
        }
      }

      if(speed == 0) {
        winchMotor.set(ControlMode.PercentOutput, 0);
        RobotContainer.xbox2.setRumble(RumbleType.kBothRumble, 0);
      }

    } else {
      winchMotor.set(ControlMode.PercentOutput, speed * ArmConstants.winchSpeed);
    }

  }

  public double getCurrentAngle() {
    return pivotCoder.getAbsolutePosition();
  }

  public double getCurrentHeight() {
    double distance = (winchCoder.getPosition()*ArmConstants.ticksPerInch);
    return winchHeight.getAverage(distance * -Math.sin(pivotCoder.getAbsolutePosition()));
  }

  public double getCurrentExtension() { //Extension relative to the frame
    double distance = (winchCoder.getPosition()*ArmConstants.ticksPerInch);
    return winchExtension.getAverage(distance * Math.cos(pivotCoder.getAbsolutePosition()));
  }

  public void grab (boolean grab) {
    if(grab) {
      grabberSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      grabberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Compressor Enabled", RobotContainer.compressor.isEnabled());
    SmartDashboard.putNumber("Winch Position", winchCoder.getPosition());
    SmartDashboard.putBoolean("Is Using Limits", RobotContainer.enableLimits);
    SmartDashboard.putNumber("Pivot Angle", getCurrentAngle());
    SmartDashboard.putNumber("Winch Height", getCurrentHeight());
    SmartDashboard.putNumber("Winch Extension", getCurrentExtension());
  }
}
