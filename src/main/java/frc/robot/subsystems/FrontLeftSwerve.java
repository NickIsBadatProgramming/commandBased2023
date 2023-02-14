// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*----- This class is used solely for smartDashboard values and to work with command-based programming */
//Most of the code for this should be in SwerveUnit

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class FrontLeftSwerve extends SwerveUnit {
  /** Creates a new FrontLeftSwerve. */
  public FrontLeftSwerve(TalonFX rotationMotor, TalonFX driveMotor, CANCoder encoder) {
    super(rotationMotor, driveMotor, encoder, true, false, SwerveConstants.FL_Zero);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Left Raw Encoder Value", this.encoder.getAbsolutePosition());
    getRawAngle();
    SmartDashboard.putNumber("Front Left Encoder Zeroed Value", this.rawAngle);

    SmartDashboard.putNumber("Drive Velocity",  this.driveMotorSpeed);
    SmartDashboard.putNumber("Steer Velocity", this.steerMotorSpeed);
    SmartDashboard.putNumber("Desired Angle FL", desiredAngle);
    SmartDashboard.putNumber("Current Angle", getRawAngle());
  }
}
