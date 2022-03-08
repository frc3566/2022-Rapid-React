// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeCamera extends SubsystemBase {

    private NetworkTableEntry lastUpdateTime_entry;

    private NetworkTableEntry processingTime_entry;
    private NetworkTableEntry fps_entry;

    private NetworkTableEntry ballDistance_entry;
    private NetworkTableEntry ballAngle_entry;

    private NetworkTableEntry ballDetected_entry;

    public IntakeCamera() {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable nt = inst.getTable("LiveWindow/IntakeCamera");
  
      lastUpdateTime_entry = nt.getEntry("last_update_time");
  
      processingTime_entry = nt.getEntry("processing_time");
      fps_entry = nt.getEntry("fps");
  
      ballDistance_entry = nt.getEntry("ball_distance");
      ballAngle_entry = nt.getEntry("ball_angle");
  
      ballDetected_entry = nt.getEntry("ball_detected");
    }

public double getLastUpdateTime(){
  return lastUpdateTime_entry.getDouble(0.0);
}

public double getProcessingTime(){ 
  return processingTime_entry.getDouble(0.0);
}

public double getTarDistance(){
  return ballDistance_entry.getDouble(0.0);
}

public double getTarAngle(){
  return ballAngle_entry.getDouble(0.0);
}

public boolean ballDetected(){
  return ballDetected_entry.getBoolean(false);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
