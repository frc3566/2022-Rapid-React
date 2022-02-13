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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intake = new CANSparkMax(10, MotorType.kBrushless);

  Compressor compressor = new Compressor(11, PneumaticsModuleType.CTREPCM);

  Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 12);

  private CANSparkMax indexer = new CANSparkMax(13, MotorType.kBrushless);

  private RelativeEncoder indexerEncoder;

  private SparkMaxPIDController indexerPID;

  private int ballCnt;


  public IntakeSubsystem() {
    intake.setInverted(false);
    intake.setClosedLoopRampRate(0.3);
    intake.setIdleMode(IdleMode.kCoast);

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

  public void enableCompressor(){
    compressor.enableDigital();
  }

  public void disableCompressor(){
    compressor.disable();
  }

  public void extendIntake(){
    solenoid.set(true);
  }

  public void contractIntake(){
    solenoid.set(false);
  }

  public void startIntake(){
      intake.set(0.3);
  }

  public void stopIntake(){
    intake.set(0);
  }

  public void startIndexer(){
    intake.set(0.3);
  }

  public void stopIndexer(){
    intake.set(0);
  }


  public void shiftIndexer(){
    //TODO measure degree turn needed
    double tarPos = 50 + indexerEncoder.getPosition();
    indexerPID.setReference(tarPos, ControlType.kPosition);
  }

  public void setBallCount(int cnt){
    ballCnt = cnt;
  }

  public int getBallCount(){
    return ballCnt;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    boolean pressureSwitch = compressor.getPressureSwitchValue();
    double current = compressor.getCurrent();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
