// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//not in use
public class LocalizationCamera extends SubsystemBase {

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable nt = inst.getTable("LiveWindow/GoalCamera");
  
  private NetworkTableEntry lastUpdateTime_entry = nt.getEntry("last_update_time");
  private NetworkTableEntry heading_entry = nt.getEntry("heading");
  
  public LocalizationCamera() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
