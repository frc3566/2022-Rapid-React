// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class GoalCamera extends SubsystemBase {

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable nt = inst.getTable("Vision");

  private NetworkTableEntry tarX;

  public GoalCamera() {}

  public double getTar(){
    tarX = nt.getEntry("target_x");
    double[] tarXs = tarX.getDoubleArray(new double[1]);

    double primaryTarX = 0.0;

    if(tarXs.length!= 0) primaryTarX = tarXs[0];
    
     // a persontage distance from the center

    // double TarAngle = (primaryTarX - x_mid) * DPP ; // angle to turn, clockwise is positive

    double tarAngle = Constants.FOV / 2 * primaryTarX;

    System.out.println(tarAngle);

    prevTarX = tarAngle;
    if(prevTarX == tarAngle){
        updated = false;
    }else{
        prevTarX = tarAngle;
        updated = true;
    }

    return tarAngle;
}

public boolean hasUpdated(){ 
    return updated;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tarX = nt.getEntry("target_x");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
