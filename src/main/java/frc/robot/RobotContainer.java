// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.BackLeftSwerve;
import frc.robot.subsystems.BackRightSwerve;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FrontLeftSwerve;
import frc.robot.subsystems.FrontRightSwerve;
import frc.robot.subsystems.SwerveGroup;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /*----- Controllers -----*/
  public static Joystick logitech3d;
  public static XboxController xbox1;


  /*----- Drive -----*/

  public static TalonFX driveFR, steerFR, driveFL, steerFL, driveBL, steerBL, driveBR, steerBR;

  public static AHRS navx;
  
  public static CANCoder cFR, cFL, cBL, cBR;

  public static Drive drive;

  /*----- Swerve Modules -----*/
  public static FrontLeftSwerve FL;
  public static FrontRightSwerve FR;
  public static BackLeftSwerve BL;
  public static BackRightSwerve BR;

  public static SwerveGroup swerve;


  

  

  



  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*----- Inputs -----*/
    xbox1 = new XboxController(0);
    //define all buttons on the XboxController here

    logitech3d = new Joystick(0);


    //Drive Motors
    driveFR = new TalonFX(1);
    driveFL = new TalonFX(3);
    driveBL = new TalonFX(5);
    driveBR = new TalonFX(7);

    //Steer Motors
    steerFR = new TalonFX(2);
    steerFL = new TalonFX(4);
    steerBL = new TalonFX(6);
    steerBR = new TalonFX(8);

    //Swerve Modules

    cFR = new CANCoder(12);
    cFL = new CANCoder(9);
    cBL = new CANCoder(10);
    cBR = new CANCoder(11);


    FR = new FrontRightSwerve(driveFR, steerFR, cFR);
    FL = new FrontLeftSwerve(driveFL, steerFL, cFL);
    BL = new BackLeftSwerve(driveBL, steerBL, cBL);
    BR = new BackRightSwerve(driveBR, steerBR, cBR);



    swerve = new SwerveGroup();
    drive = new Drive();

    //Configure CAN Settings
    cFR.setStatusFramePeriod(CANCoderStatusFrame.SensorData, SwerveConstants.RefreshRateEncoders);
    cFL.setStatusFramePeriod(CANCoderStatusFrame.SensorData, SwerveConstants.RefreshRateEncoders);
    cBL.setStatusFramePeriod(CANCoderStatusFrame.SensorData, SwerveConstants.RefreshRateEncoders);
    cBR.setStatusFramePeriod(CANCoderStatusFrame.SensorData, SwerveConstants.RefreshRateEncoders);

    driveFR.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,SwerveConstants.RefreshRateMotors);
    driveFL.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,SwerveConstants.RefreshRateMotors);
    driveBL.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,SwerveConstants.RefreshRateMotors);
    driveBR.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,SwerveConstants.RefreshRateMotors);

    steerFR.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,SwerveConstants.RefreshRateMotors);
    steerFL.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,SwerveConstants.RefreshRateMotors);
    steerBL.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,SwerveConstants.RefreshRateMotors);
    steerBR.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,SwerveConstants.RefreshRateMotors);



    //Configure neutral modes (When the motors aren't being powered);
    steerFR.setNeutralMode(NeutralMode.Brake);
    steerFL.setNeutralMode(NeutralMode.Brake);
    steerBL.setNeutralMode(NeutralMode.Brake);
    steerBR.setNeutralMode(NeutralMode.Brake);

    driveFR.setNeutralMode(NeutralMode.Brake);
    driveFL.setNeutralMode(NeutralMode.Brake);
    driveBL.setNeutralMode(NeutralMode.Brake);
    driveBR.setNeutralMode(NeutralMode.Brake);

    // Configure the button bindings
    configureButtonBindings();
  }




  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
