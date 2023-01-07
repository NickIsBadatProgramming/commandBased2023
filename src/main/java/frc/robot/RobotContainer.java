// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
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
  
  public static Encoder cFR, cFL, CBL, cBR;

  public static Drive drive;
  

  



  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = new Drive();

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





    // Configure the button bindings
    configureButtonBindings();
  }




  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}




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
