// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushless);

  private RelativeEncoder indexerEncoder;

  private SparkMaxPIDController indexerPID;

  private DigitalInput entranceIR = new DigitalInput(0);
  private DigitalInput lowIR = new DigitalInput(1);
  private DigitalInput highIR = new DigitalInput(2);

       
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable nt = inst.getTable("LiveWindow/IndexerSubsystem");
  private NetworkTableEntry ballCountEntry = nt.getEntry("ball_count");

  private int ballCnt;

  public IndexerSubsystem() {
    indexer.setInverted(false);
    indexer.setClosedLoopRampRate(0.3);
    indexer.setIdleMode(IdleMode.kCoast);

    indexerEncoder = indexer.getEncoder();
    indexerEncoder.setPositionConversionFactor(1);
    indexerEncoder.setVelocityConversionFactor(1);

    indexerPID = indexer.getPIDController();
    indexerPID.setP(0.3);
    indexerPID.setD(0.01);

    ballCnt = 1;
  }

  public void setIndexer(double power){
    indexer.set(power);
  }

  public boolean getEntranceIR(){
    return !entranceIR.get();
  }

  public boolean getLowIR(){
    return !lowIR.get();
  }

  public boolean getHighIR(){
    return !highIR.get();
  }

  public int getBallCount(){
    return ballCnt;
  }

  public void setBallCount(int cnt){
    if(cnt > 2){
      ballCnt = 2;
    }else if(cnt < 0){
      ballCnt = 0;
    }else{
      ballCnt = cnt;
    }
    return;
  }

  public void disabled(){
    indexer.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //System.out.println("Entrance: " + getEntranceIR());
    //System.out.println("Low: " + getLowIR());
    //System.out.println("High: " + getHighIR());

    ballCountEntry.setDouble(ballCnt);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
