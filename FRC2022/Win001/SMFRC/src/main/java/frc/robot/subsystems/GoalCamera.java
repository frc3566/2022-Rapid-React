// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GoalCamera extends SubsystemBase {

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable nt = inst.getTable("GoalCamera");

  private NetworkTableEntry targetEntry = nt.getEntry("angle");

  private boolean updated;

  private double prevTarX;
  private double tarAngle;

  public GoalCamera() {

    updated = true;
  }

  public double getTar(){
    updated = false;
    return tarAngle;
}

public boolean isUpdated(){ 
    return updated;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    tarAngle = targetEntry.getDouble(0.0);
    
    // System.out.println(tarAngle);

    // System.out.println(tarAngle);
    // System.out.println(updated);

    
    if(prevTarX != tarAngle){
        updated = true;
        // System.out.println(tarAngle);
    }

    prevTarX = tarAngle;
    
  }

  public void reset(){
    updated = true;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
