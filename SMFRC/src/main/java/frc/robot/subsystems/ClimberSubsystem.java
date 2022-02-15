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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  // private CANSparkMax left = new CANSparkMax(30, MotorType.kBrushless);
  // private CANSparkMax right = new CANSparkMax(31, MotorType.kBrushless);

  private RelativeEncoder leftEncoder, rightEncoder;

  private SparkMaxPIDController leftPID, rightPID;

  private boolean isExtended;
  
  public ClimberSubsystem() {
    // left.setInverted(false);
    // left.setClosedLoopRampRate(0.3);
    // left.setIdleMode(IdleMode.kBrake);

    // right.setInverted(false);
    // right.setClosedLoopRampRate(0.3);
    // right.setIdleMode(IdleMode.kBrake);

    // leftEncoder = left.getEncoder();
    // leftEncoder.setVelocityConversionFactor(1);

    // rightEncoder = right.getEncoder();
    // rightEncoder.setVelocityConversionFactor(1);

    // leftPID = left.getPIDController();
    // leftPID.setP(0.3);
    // leftPID.setI(0);
    // leftPID.setD(0.01);

    // rightPID = left.getPIDController();
    // rightPID.setP(0.3);
    // rightPID.setI(0);
    // rightPID.setD(0.01);

    isExtended = false;
  }

  public void extend(){
    if(isExtended){
      return;
    }

    //TODO measure degree turn needed
    double leftTar = leftEncoder.getPosition() + 140;
    double rightTar = leftEncoder.getPosition()  + 140;

    leftPID.setReference(leftTar, ControlType.kPosition);
    rightPID.setReference(rightTar, ControlType.kPosition);

    isExtended = true;
  }

  public void contract(){
    if(!isExtended){
      return;
    }

    double leftTar = leftEncoder.getPosition() - 140;
    double rightTar = leftEncoder.getPosition() - 140;

    // leftPID.setReference(leftTar, ControlType.kPosition);
    // rightPID.setReference(rightTar, ControlType.kPosition);

    isExtended = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
