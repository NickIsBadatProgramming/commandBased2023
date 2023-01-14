// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class FrontRightSwerve extends SwerveUnit {
  /** Creates a new FrontLeftSwerve. */
  public FrontRightSwerve(TalonFX rotationMotor, TalonFX driveMotor, CANCoder encoder) {
    super(rotationMotor, driveMotor, encoder, false, false, SwerveConstants.FR_Zero);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Front Right Raw Encoder Value", this.encoder.getAbsolutePosition());
    getRawAngle();
    SmartDashboard.putNumber("Front Right Encoder Zeroed Value", this.rawAngle);
  }
}
