// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.BalanceOnRamp;
import frc.robot.commands.GetArmToPoint;
import frc.robot.commands.GetToRamp;
import frc.robot.commands.ToggleGrip;

public class AutoManager extends SubsystemBase {

  Command aprilTag6, aprilTag7, aprilTag8, moveOutOfWay6, moveOutOfWay7, moveOutOfWay8, moveOutOfWay1, moveOutOfWay2, moveOutOfWay3;
  Command getToRamp, balanceOnRamp;
  SwerveAutoBuilder autoBuilder;
  SendableChooser<Integer> autoChooser, stationOverrideChooser;
  SequentialCommandGroup auto = new SequentialCommandGroup(new PrintCommand("Auto started"));

  public Command getToAprilTag(double tx, double tz, double az, boolean isTarget) {

    if(isTarget) {
    PathPlannerTrajectory getToTag = PathPlanner.generatePath(
      new PathConstraints(0.3, 0.2),
      new PathPoint(new Translation2d(0,0), Rotation2d.fromDegrees(0)), // initial zero position
      new PathPoint(new Translation2d(tz, tx), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(az)) //
    );
  
    return autoBuilder.fullAuto(getToTag);
    }

    return new PrintCommand("No AprilTag");
  }

  /** Creates a new AutoManager. */
  public AutoManager() {

    autoChooser = new SendableChooser<Integer>();
    autoChooser.addOption("Preload Auto", 1);
    autoChooser.addOption("Preload Auto & Move", 2);
    //autoChooser.addOption("Preload Auto & Balance", 3); //FIXMEFix balancing
    autoChooser.setDefaultOption("No Auto", -1);

    stationOverrideChooser = new SendableChooser<Integer>();
    stationOverrideChooser.addOption("1", 1);
    stationOverrideChooser.addOption("2", 2);
    stationOverrideChooser.addOption("3", 3);
    stationOverrideChooser.setDefaultOption("No Override", -1);
    

    
    //Most of this code was taken from the PathPlanner wiki


    //Auto Builder
    autoBuilder = new SwerveAutoBuilder(RobotContainer.swerve::getPose, RobotContainer.swerve::resetOdometry, new PIDConstants(5, 0, 0), new PIDConstants(0.7, 0, 0), RobotContainer.swerve::DriveChassisStates, null, false, RobotContainer.swerve);


    //Paths
    ArrayList<PathPlannerTrajectory> aprilTag6Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("aprilTag6", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> aprilTag7Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("aprilTag7", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> aprilTag8Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("aprilTag8", new PathConstraints(0.5, 0.4));

    ArrayList<PathPlannerTrajectory> moveOutOfWay6Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("moveOutOfWay6", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> moveOutOfWay7Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("moveOutOfWay7", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> moveOutOfWay8Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("moveOutOfWay8", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> moveOutOfWay1Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("moveOutOfWay1", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> moveOutOfWay2Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("moveOutOfWay2", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> moveOutOfWay3Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("moveOutOfWay3", new PathConstraints(0.5, 0.4));

    //Commands
    aprilTag6 = autoBuilder.fullAuto(aprilTag6Path);
    aprilTag7 = autoBuilder.fullAuto(aprilTag7Path);
    aprilTag8 = autoBuilder.fullAuto(aprilTag8Path);

    moveOutOfWay6 = autoBuilder.fullAuto(moveOutOfWay6Path);
    moveOutOfWay7 = autoBuilder.fullAuto(moveOutOfWay7Path);
    moveOutOfWay8 = autoBuilder.fullAuto(moveOutOfWay8Path);
    moveOutOfWay1 = autoBuilder.fullAuto(moveOutOfWay1Path);
    moveOutOfWay2 = autoBuilder.fullAuto(moveOutOfWay2Path);
    moveOutOfWay3 = autoBuilder.fullAuto(moveOutOfWay3Path);



    SmartDashboard.putData("Auto", autoChooser);

    System.out.println("Auto was built");

  }

  public Command moveArm(double angle, double extension) {
    return new GetArmToPoint(angle, extension);
  }

  public Command getAuto() {
    // //return moveArm(ArmConstants.highCubeAngle, ArmConstants.highCubeExtension);


    


    if(autoChooser.getSelected() != -1) {

      //Align with AprilTag
      // if(RobotContainer.vision.isTarget()) {
      //   auto.addCommands(getToAprilTag(RobotContainer.vision.tx(), RobotContainer.vision.tz(), RobotContainer.vision.az(), RobotContainer.vision.isTarget()));
      // }
      // auto.addCommands(new StopRobot());

      //Put cube on stair
      auto.addCommands(moveArm(ArmConstants.highCubeAngle, 0));
      auto.addCommands(new PrintCommand("got to initial angle"));
      auto.addCommands(moveArm(ArmConstants.highCubeAngle, ArmConstants.highCubeExtension));
      auto.addCommands(new PrintCommand("Got to dropping position"));
      auto.addCommands(new ToggleGrip());
      auto.addCommands(new PrintCommand("Dropped"));
      auto.addCommands(moveArm(ArmConstants.highCubeAngle, 0));
      auto.addCommands(new PrintCommand("retracted"));
      auto.addCommands(moveArm(-150, 0));
      auto.addCommands(new PrintCommand("returned to initial position"));

      if(autoChooser.getSelected() == 3) {

        if(stationOverrideChooser.getSelected() == -1) {
  
          if(DriverStation.getLocation() == 3) {
            auto.addCommands(aprilTag6);
          }
  
          if(DriverStation.getLocation() == 2 ) {
            auto.addCommands(aprilTag7);
          }
  
          if(DriverStation.getLocation() == 1 ) {
            auto.addCommands(aprilTag8);
          }
        } else {
          if(stationOverrideChooser.getSelected() == 3) {
            auto.addCommands(aprilTag6);
          }
  
          if(stationOverrideChooser.getSelected() == 2 ) {
            auto.addCommands(aprilTag7);
          }
  
          if(stationOverrideChooser.getSelected() == 1 ) {
            auto.addCommands(aprilTag8);
          }
        }
        getToRamp = new GetToRamp();
        balanceOnRamp = new BalanceOnRamp();
  
        auto.addCommands(getToRamp);
        auto.addCommands(balanceOnRamp);
  
      }
      
      if(autoChooser.getSelected() == 2) {
        if(stationOverrideChooser.getSelected() == -1) {

          if(DriverStation.getAlliance().equals(Alliance.Blue)){
              
            if(DriverStation.getLocation() == 3) {
              auto.addCommands(moveOutOfWay6);
            }
    
            if(DriverStation.getLocation() == 2 ) {
              auto.addCommands(moveOutOfWay7);
            }
    
            if(DriverStation.getLocation() == 1 ) {
              auto.addCommands(moveOutOfWay8);
            }
          } else {

            if(DriverStation.getLocation() == 3) {
              auto.addCommands(moveOutOfWay1);
            }
    
            if(DriverStation.getLocation() == 2 ) {
              auto.addCommands(moveOutOfWay2);
            }
    
            if(DriverStation.getLocation() == 1 ) {
              auto.addCommands(moveOutOfWay3);
            }
          }

        } else {

          if(DriverStation.getAlliance().equals(Alliance.Blue)){
              
            if(stationOverrideChooser.getSelected() == 3) {
              auto.addCommands(moveOutOfWay6);
            }
    
            if(stationOverrideChooser.getSelected() == 2 ) {
              auto.addCommands(moveOutOfWay7);
            }
    
            if(stationOverrideChooser.getSelected() == 1 ) {
              auto.addCommands(moveOutOfWay8);
            }
          } else {

            if(stationOverrideChooser.getSelected() == 3) {
              auto.addCommands(moveOutOfWay1);
            }
    
            if(stationOverrideChooser.getSelected() == 2 ) {
              auto.addCommands(moveOutOfWay2);
            }
    
            if(stationOverrideChooser.getSelected() == 1 ) {
              auto.addCommands(moveOutOfWay3);
            }
          }

        }
      }
    }





    return auto;

    // //return new GetToRamp();
    // SequentialCommandGroup auto = new SequentialCommandGroup(new GetToRamp());
    // auto.addCommands(new BalanceOnRamp());
    // return auto;
  }
 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
