// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;

public class SwerveUnit extends SubsystemBase {

    /*----- Independent Variables -----*/
    boolean driveInverted = false;
    boolean steerInverted = false;

    double driveMotorSpeed, steerMotorSpeed;
    double rawAngle;



    /*----- Initialize Motors and Electrics -----*/
    TalonFX rotationMotor, driveMotor;
    Encoder encoder;


  /** Creates a new SwerveUnit. */
  public SwerveUnit(TalonFX rotationMotor, TalonFX driveMotor, Encoder encoder, boolean driveInverted, boolean steerInverted) {
    this.rotationMotor = rotationMotor;
    this.driveMotor = driveMotor;
    this.encoder = encoder;
    this.driveInverted = driveInverted;
    this.steerInverted = steerInverted;
  }

  public void move(double fv, double desiredAngle) { //fv is in meters/second, desiredAngle is in degrees
    this.driveMotorSpeed = ((60 * fv)/SwerveConstants.WheelCircumferenceM) * SwerveConstants.SWERVE_GEAR_RATIO_DRIVE;
    if(driveInverted) {
      this.driveMotorSpeed = this.driveMotorSpeed * -1;
    }
    this.steerMotorSpeed = ((60 * getClosestAngle(rawAngle, desiredAngle))/(360 * SwerveConstants.MODULE_TURN_TIME_SECONDS)) * SwerveConstants.SWERVE_GEAR_RATIO_STEER;
  }


  public double getClosestAngle(double currentAngle, double desiredAngle) {
    double difference;
    if(!driveInverted) {
        difference = desiredAngle - currentAngle;
        if(Math.abs(difference) >= 180) {
          difference = 360 + difference;
          this.driveInverted = !this.driveInverted;
        }
      } else {
        difference = desiredAngle - currentAngle;
        if(Math.abs(difference) >= 180) {
          difference = difference -360;
          this.driveInverted = !this.driveInverted;
        }
    }
    return difference;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateMotorSpeeds();
    rawAngle = encoder.getDistance();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void updateMotorSpeeds() {
    driveMotor.set(ControlMode.Velocity, driveMotorSpeed * SwerveConstants.FalconSpeedConstant);
    rotationMotor.set(ControlMode.Velocity, steerMotorSpeed * SwerveConstants.FalconSpeedConstant);
  }
}
