// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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


    //Temp Stuff
    //FIXME remove when done

    SmartDashboard.putNumber("Front Left Drive Velocity", this.driveMotorSpeed);
    SmartDashboard.putNumber("Front Left Steer Velocity", this.steerMotorSpeed);
    SmartDashboard.putNumber("Desired Angle", this.desiredAngle);
    SmartDashboard.putNumber("Difference", this.difference);
  }
}
