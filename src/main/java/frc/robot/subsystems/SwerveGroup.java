// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

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
  AHRS navx;


  /** Creates a new SwerveGroup. */
  public SwerveGroup() {
    navx = new AHRS(SerialPort.Port.kMXP);

    FR = RobotContainer.FR;
    FL = RobotContainer.FL;
    BL = RobotContainer.BL;
    BR = RobotContainer.BR;
  }

  public void Drive(double vx, double vy, double vr) {
    Translation2d m_frontRight = new Translation2d(SwerveConstants.TrackwidthM/2,-SwerveConstants.WheelbaseM/2); //Making 2D translations from the center of the robot to the swerve modules
    Translation2d m_frontLeft = new Translation2d(SwerveConstants.TrackwidthM/2,SwerveConstants.WheelbaseM/2);
    Translation2d m_backLeft = new Translation2d(-SwerveConstants.TrackwidthM/2,SwerveConstants.WheelbaseM/2);
    Translation2d m_backRight = new Translation2d(-SwerveConstants.TrackwidthM/2,-SwerveConstants.WheelbaseM/2);

    SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

    ChassisSpeeds speeds;

 
    speeds = new ChassisSpeeds(vx, vy, vr); 

    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    FL.move(frontLeft.speedMetersPerSecond, frontLeft.angle.getDegrees());
    FR.move(frontRight.speedMetersPerSecond, frontRight.angle.getDegrees());
    BL.move(backLeft.speedMetersPerSecond, backLeft.angle.getDegrees());
    BR.move(backRight.speedMetersPerSecond, backRight.angle.getDegrees());

    SmartDashboard.putNumber("Front left angle", frontLeft.angle.getDegrees());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
