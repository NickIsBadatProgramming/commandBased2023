// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveGroup extends SubsystemBase {
  /*----- Initialize Swerve Modules -----*/
  static SwerveUnit FR;
  static SwerveUnit FL;
  static SwerveUnit BL;
  static SwerveUnit BR;

  static CANCoder cFR;
  static CANCoder cFL;
  static CANCoder cBL;
  static CANCoder cBR;
  AHRS navx;

  static double maxValue = 0;
  static double minValue = 0;


  /** Creates a new SwerveGroup. */
  public SwerveGroup() {
    navx = new AHRS(SerialPort.Port.kMXP);

    FR = RobotContainer.FR;
    FL = RobotContainer.FL;
    BL = RobotContainer.BL;
    BR = RobotContainer.BR;

    cFR = RobotContainer.cFR;
    cFL = RobotContainer.cFL;
    cBL = RobotContainer.cBL;
    cBR = RobotContainer.cBR;


  }

  public void Drive(double vx, double vy, double vr) { //This is being mean trying something else
    Translation2d m_frontRight = new Translation2d(SwerveConstants.TrackwidthM/2,-SwerveConstants.WheelbaseM/2); //Making 2D translations from the center of the robot to the swerve modules
    Translation2d m_frontLeft = new Translation2d(SwerveConstants.TrackwidthM/2,SwerveConstants.WheelbaseM/2);
    Translation2d m_backLeft = new Translation2d(-SwerveConstants.TrackwidthM/2,SwerveConstants.WheelbaseM/2);
    Translation2d m_backRight = new Translation2d(-SwerveConstants.TrackwidthM/2,-SwerveConstants.WheelbaseM/2);

    SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

    ChassisSpeeds speeds;

 
    speeds = new ChassisSpeeds(vx, vy, vr); 

    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(frontLeft, new Rotation2d(cFL.getAbsolutePosition()));

    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(frontRight, new Rotation2d(cFR.getAbsolutePosition()));

    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(backLeft, new Rotation2d(cBL.getAbsolutePosition()));

    SwerveModuleState backRight = moduleStates[3];
    SwerveModuleState backRightOptimized = SwerveModuleState.optimize(backRight, new Rotation2d(cBR.getAbsolutePosition()));

    FL.move(frontLeftOptimized.speedMetersPerSecond, frontLeftOptimized.angle.getDegrees());
    FR.move(frontRightOptimized.speedMetersPerSecond, frontRightOptimized.angle.getDegrees());
    BL.move(backLeftOptimized.speedMetersPerSecond, backLeftOptimized.angle.getDegrees());
    BR.move(backRightOptimized.speedMetersPerSecond, backRightOptimized.angle.getDegrees());

    SmartDashboard.putNumber("Front left angle from swerve calculation", frontLeftOptimized.angle.getDegrees());

  }


  public void CustomDrive(double x, double y, double r) {
    //Find angle of the two velocities
    //SOH CAH TOA

    double angle = Math.atan(y/x);
    if(angle < 0) {
      angle += 360; //convert negative angle to positive angle
    }

    double velocity = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    FL.move(velocity, angle);
    FR.move(velocity, angle);
    BR.move(velocity, angle);
    BL.move(velocity, angle);
  }

  public double ticksToAngle(double ticks) {
    double returnDouble = ((360d/1024d) * (ticks % 1024));
    if (ticks < 0) {
      ticks += 360;
    }

    if (returnDouble > maxValue) {
      SwerveGroup.maxValue = returnDouble;
    }
    if (returnDouble < minValue) {
      SwerveGroup.minValue = returnDouble;
    }
    return returnDouble;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Max Angle", maxValue);
    SmartDashboard.putNumber("Min Angle", minValue);
  }
}
