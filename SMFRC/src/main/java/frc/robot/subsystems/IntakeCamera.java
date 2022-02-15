// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ballColors;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeCamera extends SubsystemBase {

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable nt = inst.getTable("IntakeCamera");

    private NetworkTableEntry lastUpdateTime_entry = nt.getEntry("last_update_time");

    private NetworkTableEntry processingTime_entry = nt.getEntry("processing_time");
    private NetworkTableEntry fps_entry = nt.getEntry("fps");

    private NetworkTableEntry ballDistance_entry = nt.getEntry("ball_distance");
    private NetworkTableEntry ballAngle_entry = nt.getEntry("ball_angle");

    public IntakeCamera() {
      
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

public boolean ballDetected(){
  if(Constants.ballColor == ballColors.RED && getRedList().length != 0){
    return true;
  }else if(Constants.ballColor == ballColors.BLUE && getBlueList().length != 0){
    return true;
  }
    return false;
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
