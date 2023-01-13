// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;

public class SwerveUnit extends SubsystemBase {

    /*----- Independent Variables -----*/
    boolean driveInverted = false;
    boolean steerInverted = false;

    double driveMotorSpeed, steerMotorSpeed;
    double rawAngle;

    static double maxDesired = 0;
    static double minDesired = 0;



    /*----- Initialize Motors and Electrics -----*/
    TalonFX rotationMotor, driveMotor;
    CANCoder encoder;


  /** Creates a new SwerveUnit. */
  public SwerveUnit(TalonFX driveMotor, TalonFX rotationMotor, CANCoder encoder, boolean driveInverted, boolean steerInverted) {
    this.rotationMotor = rotationMotor;
    this.driveMotor = driveMotor;
    this.encoder = encoder;
    this.driveInverted = driveInverted;
    this.steerInverted = steerInverted;
  }

  public void move(double fv, double desiredAngle) { //fv is in meters/second, desiredAngle is in degrees

    this.driveMotorSpeed = fv * SwerveConstants.SpeedMultiplier; //((60 * fv)/SwerveConstants.WheelCircumferenceM) //* SwerveConstants.SWERVE_GEAR_RATIO_DRIVE;
    //this.steerMotorSpeed = ((60 * getClosestAngle(ticksToAngle(encoder.getDistance()), desiredAngle))/(360 * SwerveConstants.MODULE_TURN_TIME_SECONDS)) * SwerveConstants.SWERVE_GEAR_RATIO_STEER;
    this.steerMotorSpeed = steerSpeed(encoder.getAbsolutePosition(), desiredAngle);

    //Diagnostic stuff

    if (desiredAngle > maxDesired) {
      SwerveUnit.maxDesired = desiredAngle;
    }
    if (desiredAngle < minDesired) {
      SwerveUnit.minDesired = desiredAngle;
    }
    SmartDashboard.putNumber("desired angle", desiredAngle);
    SmartDashboard.putNumber("Max Desired", maxDesired);
    SmartDashboard.putNumber("Min Desired", minDesired);
  }


  /*public double getClosestAngle(double currentAngle, double desiredAngle) {
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
  } */

  public double steerSpeed(double currentAngle, double desiredAngle) {
    //at 0 degrees from the angle we want he rotation speed to be 0
    // at 180 degrees we have a constant in constants for our maximum speed
    //these two points can make a linear graph

    //return SwerveConstants.MaxModuleTurnSpeed * ((currentAngle-desiredAngle)/180);

    double difference = desiredAngle-currentAngle;
    if(difference > 180) {
      difference -= 360;
    }

    if(difference < -180) {
      difference += 360;
    }

    if(Math.abs(difference) <= 4) {
      return 0;
    } 
    if(Math.abs(desiredAngle-currentAngle) <= 10) {
      return (SwerveConstants.MaxModuleTurnSpeed * ((difference)/Math.abs(difference)))/2;
    } 
    
    return SwerveConstants.MaxModuleTurnSpeed * ((difference)/Math.abs(difference));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double ticksToAngle(double ticks) {
    double returnDouble = ((360d/1024d) * (ticks % 1024));
    if (returnDouble < 0) {
      returnDouble += 360;
    }
    return returnDouble;
  }

  public void updateMotorSpeeds() {
    //driveMotor.set(ControlMode.PercentOutput, driveMotorSpeed);
    //rotationMotor.set(ControlMode.PercentOutput, steerMotorSpeed);
    SmartDashboard.putNumber("steer motor speed", steerMotorSpeed);
    SmartDashboard.putNumber("Drive Motor Speed", driveMotorSpeed);
  }
}
