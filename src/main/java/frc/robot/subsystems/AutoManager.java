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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.BalanceOnRamp;
import frc.robot.commands.Drive;
import frc.robot.commands.GetArmToPoint;
import frc.robot.commands.GetToRamp;
import frc.robot.commands.StopRobot;
import frc.robot.commands.ToggleGrip;

public class AutoManager extends SubsystemBase {

  Command aprilTag6, aprilTag7, aprilTag8;
  SwerveAutoBuilder autoBuilder;
  SendableChooser<Integer> autoChooser;
  SendableChooser<Boolean> balanceChooser;
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
    autoChooser.addOption("Preload Auto Right", 1);
    autoChooser.addOption("Preload Auto Middle", 2);
    autoChooser.addOption("Preload Auto Left", 3);
    autoChooser.setDefaultOption("No Auto", -1);


    balanceChooser = new SendableChooser<Boolean>();
    balanceChooser.addOption("Balance", true);
    balanceChooser.setDefaultOption("Don't Balance", false);

    
    //Most of this code was taken from the PathPlanner wiki


    //Auto Builder
    autoBuilder = new SwerveAutoBuilder(RobotContainer.swerve::getPose, RobotContainer.swerve::resetOdometry, new PIDConstants(5, 0, 0), new PIDConstants(0.7, 0, 0), RobotContainer.swerve::DriveChassisStates, null, true, RobotContainer.swerve);


    //Paths
    ArrayList<PathPlannerTrajectory> aprilTag6Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("aprilTag6", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> aprilTag7Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("aprilTag7", new PathConstraints(0.5, 0.4));
    ArrayList<PathPlannerTrajectory> aprilTag8Path = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("aprilTag8", new PathConstraints(0.5, 0.4));

    //Commands
    aprilTag6 = autoBuilder.fullAuto(aprilTag6Path);
    aprilTag7 = autoBuilder.fullAuto(aprilTag7Path);
    aprilTag8 = autoBuilder.fullAuto(aprilTag8Path);

    SmartDashboard.putData("Balance", balanceChooser);
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
    }

    if(balanceChooser.getSelected()) {

      if(autoChooser.getSelected() == 1 ) {
        auto.addCommands(aprilTag6);
      }

      if(autoChooser.getSelected() == 2 ) {
        auto.addCommands(aprilTag7);
      }

      if(autoChooser.getSelected() == 3 ) {
        auto.addCommands(aprilTag8);
      }

      auto.addCommands(new GetToRamp());
      auto.addCommands(new BalanceOnRamp());

    }

    return auto;

    // //return new GetToRamp();
    // SequentialCommandGroup auto = new SequentialCommandGroup(new GetToRamp());
    // auto.addCommands(new BalanceOnRamp());
    // return auto;
  }

  public void cancel() {
    auto.cancel();
  }
 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
