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

  // Compressor compressor = new Compressor(11, PneumaticsModuleType.CTREPCM);

  // private CANSparkMax indexer = new CANSparkMax(12, MotorType.kBrushless);

  // Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  private int ballCnt;


  public IntakeSubsystem() {
    intake.setInverted(true);
    intake.setClosedLoopRampRate(0.3);
    intake.setIdleMode(IdleMode.kCoast);

    ballCnt = 1;
  }

  public void enableCompressor(){
    // compressor.enableDigital();
  }

  public void disableCompressor(){
    // compressor.disable();
  }

  public void extendIntake(){
    // solenoid.set(true);
  }

  public void contractIntake(){
    // solenoid.set(false);
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
