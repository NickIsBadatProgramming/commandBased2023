// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollingAverage extends SubsystemBase {

  private List<Double> averages;
  private int historyLimit;


  /** Creates a new RollingAverage. */
  public RollingAverage(int historyLimit) {
    this.historyLimit = historyLimit - 1;
    averages = new ArrayList<Double>();
  }

  public double getAverage(double kValue) {
    averages.add(kValue);
    if(averages.size() > historyLimit) {
      averages.remove(averages.size()-historyLimit);
    }
    
    double sum = 0;
    for(int i = 0; i < averages.size(); i++) {
      sum += averages.get(i);
    }

    return (sum/(averages.size() + 1));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
