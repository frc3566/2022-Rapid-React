// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushless);

  private RelativeEncoder indexerEncoder;

  private SparkMaxPIDController indexerPID;

  DigitalInput entranceIR = new DigitalInput(0);
  DigitalInput lowIR = new DigitalInput(1);
  DigitalInput highIR = new DigitalInput(2);

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

    ballCnt = 0;
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
    ballCnt = cnt;
    return;
  }

  public void disabled(){
    indexer.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("Entrance: " + entranceIR.get());
    // System.out.println("Low: " + lowIR.get());
    // System.out.println("High: " + highIR.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
