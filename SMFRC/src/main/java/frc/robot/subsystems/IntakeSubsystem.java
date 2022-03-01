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

  Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
  Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4);

  private int ballCnt;
  private boolean isExtented;


  public IntakeSubsystem() {
    intake.setInverted(false);
    intake.setClosedLoopRampRate(0.3);
    intake.setIdleMode(IdleMode.kCoast);

    ballCnt = 1;
    isExtented = false;
  }

  public void enableCompressor(){
    compressor.enableDigital();
  }

  public void disableCompressor(){
    compressor.disable();
  }

  public void extendIntake(){
    leftSolenoid.set(true);
    rightSolenoid.set(true);
    isExtented = true;
  }

  public void contractIntake(){
    leftSolenoid.set(false);
    rightSolenoid.set(false);
    isExtented = false;
  }

  public void toggleIntake(){
    if(isExtented){
      contractIntake();
    }else{
      extendIntake();
    }
  }

  public void setIntake(double power){
    intake.set(power);
  }

  public void setBallCount(int cnt){
    ballCnt = cnt;
  }

  public int getBallCount(){
    return ballCnt;
  }

  public void disabled(){
    intake.set(0);
    this.contractIntake();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // boolean pressureSwitch = compressor.getPressureSwitchValue();
    // double current = compressor.getCurrent();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
