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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AutoManager extends SubsystemBase {
  /** Creates a new AutoManager. */
  public AutoManager() {
    
    //Most of this code was taken from the PathPlanner wiki


    //Auto Builder
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(RobotContainer.swerve::getPose, RobotContainer.swerve::resetOdometry, new PIDConstants(5, 0, 0), new PIDConstants(0.5, 0, 0), RobotContainer.swerve::DriveChassisStates, null, RobotContainer.swerve);

    //Paths
    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>)PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

    //Commands
    Command fullAuto = autoBuilder.fullAuto(pathGroup);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
