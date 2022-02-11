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

    private NetworkTableEntry lastUpdateTime_Entry = nt.getEntry("last_update_time");

    private NetworkTableEntry processingTime_Entry = nt.getEntry("processing_time");
    private NetworkTableEntry fps_Entry = nt.getEntry("fps");

    private NetworkTableEntry redBallDistance_Entry = nt.getEntry("red_ball_distance");
    private NetworkTableEntry redBallAngle_Entry = nt.getEntry("red_ball_angle");

    private NetworkTableEntry redBallXList_Entry = nt.getEntry("red_ball_x_list");
    private NetworkTableEntry redBallYList_Entry = nt.getEntry("red_ball_y_list");

    private NetworkTableEntry blueBallDistance_Entry = nt.getEntry("blue_ball_distance");
    private NetworkTableEntry blueBallAngle_Entry = nt.getEntry("blue_ball_angle");

    private NetworkTableEntry blueBallXList_Entry = nt.getEntry("blue_ball_x_list");
    private NetworkTableEntry blueBallYList_Entry = nt.getEntry("blue_ball_y_list");

    private boolean updated;

    public IntakeCamera() {

    updated = false;
    }

public boolean isUpdated(){ 
    return updated;
}

public double getLastUpdateTime(){
  return lastUpdateTime_Entry.getDouble(0);
}

public double getProcessingTime(){ 
    return processingTime_Entry.getDouble(0);
}

public double getTarDistance(){
  if(Constants.ballColor == ballColors.RED){
    return redBallDistance_Entry.getDouble(0);

  }else if(Constants.ballColor == ballColors.BLUE){
    return blueBallDistance_Entry.getDouble(0);
    
  }
  return 0.0;
}

public double getTarAngle(){
  if(Constants.ballColor == ballColors.RED){
    return redBallAngle_Entry.getDouble(0);

  }else if(Constants.ballColor == ballColors.BLUE){
    return blueBallAngle_Entry.getDouble(0);

  }
  return 0.0;
}

public double[][] getRedList(){
  double[] xList = redBallXList_Entry.getDoubleArray(new double[0]);
  double[] yList = redBallYList_Entry.getDoubleArray(new double[0]);

  double[][] ret = new double[xList.length][2];

  for(int i=0; i<xList.length; i++){
    ret[i][0] = xList[1];
    ret[i][1] = yList[1];
  }

  return ret;
}

public double[][] getBlueList(){
  double[] xList = blueBallXList_Entry.getDoubleArray(new double[0]);
  double[] yList = blueBallYList_Entry.getDoubleArray(new double[0]);

  double[][] ret = new double[xList.length][2];

  for(int i=0; i<xList.length; i++){
    ret[i][0] = xList[1];
    ret[i][1] = yList[1];
  }

  return ret;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //TODO put on shuffle board

    
  }

  public void reset(){
    updated = true;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
