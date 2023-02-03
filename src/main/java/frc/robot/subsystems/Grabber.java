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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class Grabber extends SubsystemBase {

  public static TalonFX winchMotor, pivotMotor;
  public static PneumaticHub pneumaticHub;
  public static DoubleSolenoid grabRight, grabLeft, brake;

  public static boolean isBrake = false;

  
  /** Creates a new Grabber. */
  public Grabber() {
    winchMotor = RobotContainer.winchMotor;
    pivotMotor = RobotContainer.pivotMotor;
    pneumaticHub = RobotContainer.pneumaticHub;
    grabRight = RobotContainer.grabRight;
    grabLeft = RobotContainer.grabLeft;
    brake = RobotContainer.brake;
  }

  public void pivot (int direction) { //This should be -1 or 1
    pivotMotor.set(ControlMode.PercentOutput, direction * ArmConstants.pivotSpeed);
  }

  public void winch (int direction) { //This should be -1 or 1
    winchMotor.set(ControlMode.PercentOutput, direction * ArmConstants.pivotSpeed);
  }

  public void brake () {
    brake.set(DoubleSolenoid.Value.kForward);
  }
  public void unBrake () {
    brake.set(DoubleSolenoid.Value.kReverse);
  }

  public void grab () {
    grabRight.set(DoubleSolenoid.Value.kForward);
    grabLeft.set(DoubleSolenoid.Value.kForward);
  }

  public void unGrab () {
    grabRight.set(DoubleSolenoid.Value.kReverse);
    grabLeft.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
