// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.Constants;

public class ShooterCamera extends SubsystemBase {

  private NetworkTableEntry lastUpdateTime_entry;

  private NetworkTableEntry processingTime_entry;
  private NetworkTableEntry fps_entry;

  private NetworkTableEntry goalDetected_entry;

  private NetworkTableEntry xAngle_entry;
  private NetworkTableEntry yAngle_entry;

  private NetworkTableEntry distance_entry;

  private NetworkTableEntry PredictedRPMEntry;

  private double prevUpdateTime = 0;

  public enum LastSeen {LEFT, RIGHT};

  public LastSeen lastSeen = LastSeen.LEFT;

  private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
  interpolator = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(100);


  public ShooterCamera() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable nt = inst.getTable("LiveWindow/ShooterCamera");
  
    lastUpdateTime_entry = nt.getEntry("last_update_time");
  
    processingTime_entry = nt.getEntry("processing_time");
    fps_entry = nt.getEntry("fps");
  
    goalDetected_entry = nt.getEntry("goal_detected");
  
    xAngle_entry = nt.getEntry("x_angle");
    yAngle_entry = nt.getEntry("y_angle");
  
    distance_entry = nt.getEntry("distance");

    PredictedRPMEntry = nt.getEntry("predicted_RPM");

    for(double[] t : Constants.shooterData){
      interpolator.put(new InterpolatingDouble(t[0]), new InterpolatingDouble(t[1]));
  }

  }

  public double getLastUpdateTime(){
    return lastUpdateTime_entry.getDouble(0.0);
  }
  
  public double getProcessingTime(){ 
      return processingTime_entry.getDouble(0.0);
  }
  
  public double getTarDistance(){
    return distance_entry.getDouble(0.0);
  }

  public double getTarAngle(){
    return xAngle_entry.getDouble(0.0);
  }

  public boolean goalDetected(){
    return goalDetected_entry.getDouble(0.0) == 1.0;
  }

  public double distanceToRPM(double distance){
    return interpolator.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  double tarAngle = getTarAngle();
    if(goalDetected()){
      if(tarAngle < 0){
        lastSeen = LastSeen.LEFT;
      }else{
        lastSeen = LastSeen.RIGHT;
      }
    }

    PredictedRPMEntry.setDouble(distanceToRPM(getTarDistance()));

    // System.out.println(goalDetected());

    // System.out.println(tarAngle);

    // System.out.println(tarAngle);
    // System.out.println(updated);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
