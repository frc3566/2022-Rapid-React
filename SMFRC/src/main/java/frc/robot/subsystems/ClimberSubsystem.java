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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax left = new CANSparkMax(30, MotorType.kBrushless);
  private CANSparkMax right = new CANSparkMax(31, MotorType.kBrushless);

  private RelativeEncoder leftEncoder, rightEncoder;

  private SparkMaxPIDController leftPID, rightPID;

  private boolean isExtended;

  private double leftZero, rightZero;

  private double protectionDelayTarget;

  private boolean calibrationMode = false;

  private double minSpeed = 10000;
  
  public ClimberSubsystem() {
    left.setInverted(false);
    left.setClosedLoopRampRate(0.3);
    left.setIdleMode(IdleMode.kBrake);

    right.setInverted(false);
    right.setClosedLoopRampRate(0.3);
    right.setIdleMode(IdleMode.kBrake);

    leftEncoder = left.getEncoder();
    leftEncoder.setVelocityConversionFactor(1);

    rightEncoder = right.getEncoder();
    rightEncoder.setVelocityConversionFactor(1);

    leftPID = left.getPIDController();
    leftPID.setP(0.3);
    leftPID.setI(0);
    leftPID.setD(0.01);

    rightPID = left.getPIDController();
    rightPID.setP(0.3);
    rightPID.setI(0);
    rightPID.setD(0.01);

    isExtended = false;

    setZero();
  }

  public void setCalibrationMode(boolean mode){
    calibrationMode = mode;
  }

  public void setPower(double power){
    left.set(power);
    right.set(power);
  }

  public void setLeft(double power){
    left.set(power);
  }

  public void setRight(double power){
    right.set(power);
  }

  public void extend(){
    if(isExtended){
      return;
    }

    double leftTar = leftEncoder.getPosition() + 180;
    double rightTar = leftEncoder.getPosition()  + 180;

    leftPID.setReference(leftTar, ControlType.kPosition);
    rightPID.setReference(rightTar, ControlType.kPosition);

    isExtended = true;
  }

  public void contract(){
    if(!isExtended){
      return;
    }

    double leftTar = leftEncoder.getPosition() - 180;
    double rightTar = leftEncoder.getPosition() - 190;

    // leftPID.setReference(leftTar, ControlType.kPosition);
    // rightPID.setReference(rightTar, ControlType.kPosition);

    isExtended = false;
  }

  public void setZero(){
    leftZero = leftEncoder.getPosition();
    rightZero = rightEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // soft pertection
    if(!calibrationMode){
      if(leftEncoder.getPosition() <= leftZero && left.get() < 0){
        left.set(0);
      }
      if(leftEncoder.getPosition() >= leftZero + 180 && left.get() > 0){
        left.set(0);
      }
      if(rightEncoder.getPosition() <= rightZero && right.get() < 0){
        right.set(0);
      }
      if(rightEncoder.getPosition() >= rightZero + 180 && right.get() > 0){
        right.set(0);
      }
    }


    // if(Timer.getFPGATimestamp() > protectionDelayTarget && Math.abs(leftEncoder.getVelocity()) < minSpeed){
    //   left.set(0);
    // }
    // if(Timer.getFPGATimestamp() > protectionDelayTarget && Math.abs(rightEncoder.getVelocity()) < minSpeed){
    //   right.set(0);
    // }

    // if(left.get() == 0 || right.get() == 0){
    //   protectionDelayTarget = Timer.getFPGATimestamp() + 0.5;
    // }

    // System.out.println("left velocity: " + leftEncoder.getVelocity());
    // System.out.println("right velocity: " + rightEncoder.getVelocity());

    // System.out.println("left position: " + leftEncoder.getPosition());
    // System.out.println("right position: " + rightEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
