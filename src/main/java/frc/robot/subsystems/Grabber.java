// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//POSITIONAL DEFINITIONS

//0 Degrees = grabber is facing forwards parallel to the robot frame
//0 Extension = grabber is at minimum extension 


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class Grabber extends SubsystemBase {

  public static TalonFX winchMotor, pivotMotor;
  public static PneumaticHub pneumaticHub;
  public static DoubleSolenoid grabberSolenoid;
  public static DoubleSolenoid brakeSolenoid;

  public static boolean isBrake = false;

  
  /** Creates a new Grabber. */
  public Grabber() {
    
    winchMotor = RobotContainer.winchMotor;
    pivotMotor = RobotContainer.pivotMotor;
    pneumaticHub = RobotContainer.pneumaticHub;
    grabberSolenoid = RobotContainer.grabberSolenoid;
    brakeSolenoid = RobotContainer.brakeSolenoid;
  }

  public void pivot (double speed) { //This should be -1 or 1
    double position = pivotMotor.getSelectedSensorPosition();
    //speed = -speed;

    if(RobotContainer.enableLimits) {

      if(speed == 0) {
        pivotMotor.set(ControlMode.PercentOutput, 0);
        brakeSolenoid.set(DoubleSolenoid.Value.kForward);
      } else {
        brakeSolenoid.set(DoubleSolenoid.Value.kReverse);
      }

      if(speed < 0) {
        if(position < ArmConstants.MinArmPivot + ArmConstants.PivotError) {
          pivotMotor.set(ControlMode.PercentOutput, 0);
        } else {
          pivotMotor.set(ControlMode.PercentOutput, speed * ArmConstants.pivotSpeed);
        }
      } 

      if(speed > 0) {
        if(position > ArmConstants.MaxArmPivot - ArmConstants.PivotError) {
          winchMotor.set(ControlMode.PercentOutput, 0);
        } else {
          pivotMotor.set(ControlMode.PercentOutput, speed * ArmConstants.pivotSpeed);
        }
      }




    } else {
      pivotMotor.set(ControlMode.PercentOutput, speed * ArmConstants.winchSpeed);
    }
  }

  public void winch (double speed) { //This should be -1 or 1

    speed = -speed;
    double position = winchMotor.getSelectedSensorPosition();

    if(RobotContainer.enableLimits) {

      if(speed < 0) {
        if(position < ArmConstants.MinArmLength + ArmConstants.ArmLengthError) {
          winchMotor.set(ControlMode.PercentOutput, 0);
        } else {
          winchMotor.set(ControlMode.PercentOutput, speed * ArmConstants.winchSpeed);
        }
      } 

      if(speed > 0) {
        if(position > ArmConstants.MaxArmLength - ArmConstants.ArmLengthError) {
          winchMotor.set(ControlMode.PercentOutput, 0);
        } else {
          winchMotor.set(ControlMode.PercentOutput, speed * ArmConstants.winchSpeed);
        }
      }

      if(speed == 0) {
        winchMotor.set(ControlMode.PercentOutput, 0);
      }

    } else {
      winchMotor.set(ControlMode.PercentOutput, speed * ArmConstants.winchSpeed);
    }

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
    SmartDashboard.putBoolean("Compressor Enabled", RobotContainer.compressor.enabled());
    SmartDashboard.putNumber("Winch Position", winchMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Is Using Limits", RobotContainer.enableLimits);
    SmartDashboard.putNumber("Pivot Angle", pivotMotor.getSelectedSensorPosition());
  }
}
