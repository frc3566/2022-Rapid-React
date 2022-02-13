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

    private NetworkTableEntry redBallDistance_entry = nt.getEntry("red_ball_distance");
    private NetworkTableEntry redBallAngle_entry = nt.getEntry("red_ball_angle");

    private NetworkTableEntry redBallXList_entry = nt.getEntry("red_ball_x_list");
    private NetworkTableEntry redBallYList_entry = nt.getEntry("red_ball_y_list");

    private NetworkTableEntry blueBallDistance_entry = nt.getEntry("blue_ball_distance");
    private NetworkTableEntry blueBallAngle_entry = nt.getEntry("blue_ball_angle");

    private NetworkTableEntry blueBallXList_entry = nt.getEntry("blue_ball_x_list");
    private NetworkTableEntry blueBallYList_entry = nt.getEntry("blue_ball_y_list");

    public IntakeCamera() {
      
    }

public double getLastUpdateTime(){
  return lastUpdateTime_entry.getDouble(0.0);
}

public double getProcessingTime(){ 
    return processingTime_entry.getDouble(0.0);
}

public double getTarDistance(){
  if(Constants.ballColor == ballColors.RED){
    return redBallDistance_entry.getDouble(0.0);

  }else if(Constants.ballColor == ballColors.BLUE){
    return blueBallDistance_entry.getDouble(0.0);
    
  }
  return 0.0;
}

public double getTarAngle(){
  if(Constants.ballColor == ballColors.RED){
    return redBallAngle_entry.getDouble(0.0);

  }else if(Constants.ballColor == ballColors.BLUE){
    return blueBallAngle_entry.getDouble(0.0);

  }
  return 0.0;
}

public double[][] getRedList(){
  double[] xList = redBallXList_entry.getDoubleArray(new double[0]);
  double[] yList = redBallYList_entry.getDoubleArray(new double[0]);

  double[][] ret = new double[xList.length][2];

  for(int i=0; i<xList.length; i++){
    ret[i][0] = xList[1];
    ret[i][1] = yList[1];
  }

  return ret;
}

public double[][] getBlueList(){
  double[] xList = blueBallXList_entry.getDoubleArray(new double[0]);
  double[] yList = blueBallYList_entry.getDoubleArray(new double[0]);

  double[][] ret = new double[xList.length][2];

  for(int i=0; i<xList.length; i++){
    ret[i][0] = xList[1];
    ret[i][1] = yList[1];
  }

  return ret;
}

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

    //TODO put on shuffle board

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
