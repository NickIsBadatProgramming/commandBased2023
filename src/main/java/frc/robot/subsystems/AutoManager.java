// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AutoManager extends SubsystemBase {

  Command fullAuto;


  /** Creates a new AutoManager. */
  public AutoManager() {

    SendableChooser<Integer> autoChooser = new SendableChooser<Integer>();
    autoChooser.addOption("Preload Auto", 1);
    autoChooser.addOption("Preload + 1 Auto", 2);
    autoChooser.setDefaultOption("No Auto", -1);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SendableChooser<Boolean> balanceChooser = new SendableChooser<Boolean>();
    balanceChooser.addOption("Balance", false);
    balanceChooser.setDefaultOption("Don't Balance", true);
    SmartDashboard.putData(balanceChooser);
    
    //Most of this code was taken from the PathPlanner wiki


    //Auto Builder
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(RobotContainer.swerve::getPose, RobotContainer.swerve::resetOdometry, new PIDConstants(5, 0, 0), new PIDConstants(0.5, 0, 0), RobotContainer.swerve::DriveChassisStates, null, RobotContainer.swerve);

    //Paths
    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("FullAuto2", new PathConstraints(0.2, 0.2));

    //Commands
    fullAuto = autoBuilder.fullAuto(pathGroup);

    System.out.println("Auto was built");

  }

  public Command getAuto() {
    return fullAuto;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
