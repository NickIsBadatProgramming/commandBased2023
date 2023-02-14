// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Config;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveCoordinates;
import frc.robot.commands.EnableLimts;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Grab;
import frc.robot.commands.ResetFeild;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.ToggleGrip;
import frc.robot.commands.UseField;
import frc.robot.commands.VisionStatus;
import frc.robot.commands.ZeroArm;
import frc.robot.subsystems.BackLeftSwerve;
import frc.robot.subsystems.BackRightSwerve;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FrontLeftSwerve;
import frc.robot.subsystems.FrontRightSwerve;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.SwerveGroup;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //


  /*----- Controllers -----*/
  public static Joystick logitech3d;
  public static JoystickButton thumbButton;
  public static JoystickButton button5;






  public static XboxController xbox1, xbox2;
  public static JoystickButton xbox1SS; //Screen Share button, two rectangles
  public static JoystickButton xbox1Settings;
  public static JoystickButton backRightPaddle;

  //xbox2

  public static JoystickButton xbox2A;
  public static JoystickButton xbox2B;
  public static JoystickButton xbox2X;



  /*----- Drive -----*/

  public static TalonFX driveFR, steerFR, driveFL, steerFL, driveBL, steerBL, driveBR, steerBR;

  public static AHRS navx;
  
  public static CANCoder cFR, cFL, cBL, cBR;

  public static Drive drive;
  public static boolean isUsingField = true;

  /*----- Swerve Modules -----*/
  public static FrontLeftSwerve FL;
  public static FrontRightSwerve FR;
  public static BackLeftSwerve BL;
  public static BackRightSwerve BR;

  public static SwerveGroup swerve;
  public static ResetFeild resetFeild;
  public static UseField useField;
  public static Vision vision;
  public static VisionStatus status;
  public static ResetOdometry resetOdometry;
  public static DriveCoordinates driveCoordinates;


  /*----- Gripper Components -----*/ 
  public static TalonFX winchMotor, pivotMotor;
  public static PneumaticHub pneumaticHub;
  public static DoubleSolenoid grabberSolenoid, brakeSolenoid;
  public static Solenoid test;

  public static boolean enableLimits = true;
  public static EnableLimts enableLimitsCommand;  
  public static Grabber grabber;
  public static ZeroArm zeroArm;
  public static Compressor compressor;
  public static Grab grabCommand;
  public static ToggleGrip toggleGrip;
  public static boolean grip = false;

  public static CANCoder winchCAN, pivotCAN;

  public static double winchZero = 0;
  public static double pivotZero = 0;


  



  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    /*----- Inputs -----*/
    xbox1 = new XboxController(0);
    xbox1SS = new JoystickButton(xbox1,7);
    xbox1Settings = new JoystickButton(xbox1,8);
    backRightPaddle = new JoystickButton(xbox1,6);
    //define all buttons on the XboxController here


    //Logitech flight stick
    logitech3d = new Joystick(0);
    thumbButton = new JoystickButton(logitech3d, 2);
    button5 = new JoystickButton(logitech3d, 5);

    //xbox controller #2
    xbox2 = new XboxController(1);
    xbox2A = new JoystickButton(xbox2, 1);
    xbox2B = new JoystickButton(xbox2, 2);
    xbox2X = new JoystickButton(xbox2, 3);





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

    navx = new AHRS(SerialPort.Port.kMXP);


    FR = new FrontRightSwerve(driveFR, steerFR, cFR);
    FL = new FrontLeftSwerve(driveFL, steerFL, cFL);
    BL = new BackLeftSwerve(driveBL, steerBL, cBL);
    BR = new BackRightSwerve(driveBR, steerBR, cBR);



    swerve = new SwerveGroup();
    drive = new Drive();
    resetFeild = new ResetFeild();
    useField = new UseField();
    vision = new Vision();
    status = new VisionStatus();
    resetOdometry = new ResetOdometry();
    driveCoordinates = new DriveCoordinates(0.6, 0, 0); //Y is (-) left and (+) right and X is (-) back to (+) front


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

    //SmartDashboard Stuff
    SmartDashboard.putData("Reset Field", resetFeild);
    SmartDashboard.putData("Switch Relativity", useField);
    SmartDashboard.putData("Reset Odometry", resetOdometry);

    //Gripper Stuff

    
    pneumaticHub = new PneumaticHub(17);
    grabberSolenoid = new DoubleSolenoid(17,PneumaticsModuleType.CTREPCM, 0, 1); //FIXME add the right channels
    brakeSolenoid = new DoubleSolenoid(17, PneumaticsModuleType.CTREPCM, 2, 3);
    pivotMotor = new TalonFX(16);
    winchMotor = new TalonFX(15);
    enableLimitsCommand = new EnableLimts();
    grabber = new Grabber();
    grabCommand = new Grab();
    toggleGrip = new ToggleGrip();

    pivotMotor.setNeutralMode(NeutralMode.Brake);
    winchMotor.setNeutralMode(NeutralMode.Brake);

    compressor = new Compressor(17, PneumaticsModuleType.CTREPCM);
    System.out.println("Compressor initated as " + compressor.getConfigType());

    SmartDashboard.putData("Limit Switch", enableLimitsCommand);
    zeroArm = new ZeroArm();
    winchCAN = new CANCoder(18); //FIXME add right numbers
    //pivotCAN = new CANCoder(0);

    
    

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
    if(Config.usingLogitech360) {
      thumbButton.whenPressed(resetFeild);
      button5.whenPressed(useField);
    } else {
      xbox1SS.whenPressed(resetFeild);
      xbox1Settings.whenPressed(useField);
    }

    xbox2A.whenPressed(toggleGrip);
    xbox2B.whenPressed(enableLimitsCommand);
    xbox2X.whenPressed(zeroArm);

  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return driveCoordinates;
  }
}
