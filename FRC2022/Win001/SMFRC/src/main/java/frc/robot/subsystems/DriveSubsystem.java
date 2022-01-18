// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DriveSignal;
import frc.robot.util.Util;

public class DriveSubsystem extends SubsystemBase {
  private TimerSubsystem timer;
  
  private CANSparkMax left1;
  private WPI_TalonSRX left2, left3;

  private CANSparkMax right1;
  private WPI_TalonSRX right2, right3;

  private RelativeEncoder leftEncoder, rightEncoder;

  private PigeonIMU IMU;

  private double currGyro;
  private double gyroZero;
  public double tarGyro;
  private double prevGyroError;

  private SparkMaxPIDController leftController, rightController;

  public enum DriveControlState {
    VOLTAGE_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
  }

  private DriveControlState driveControlState = DriveControlState.POSITION_CONTROL;

  public DriveSubsystem(TimerSubsystem timerSubsystem) {
    this.timer = timerSubsystem;

    left1 = new CANSparkMax(12, MotorType.kBrushless);
    setSpark(left1, false);
    leftController = left1.getPIDController();
    setControler(leftController);

    left2 = new WPI_TalonSRX(3);
    setTalon(left2, false);

    left3 = new WPI_TalonSRX(4);
    setTalon(left3, false);

    right1 = new CANSparkMax(11, MotorType.kBrushless);
    setSpark(right1, true);
    rightController = right1.getPIDController();
    setControler(rightController);

    right2 = new WPI_TalonSRX(1);
    setTalon(right2, false);

    right3 = new WPI_TalonSRX(2);
    setTalon(right3, true);

    rightEncoder = right1.getEncoder();
    leftEncoder = left1.getEncoder();

    IMU = new PigeonIMU(right2);

    setGyroZero();
    getGyro();

    tarGyro = 0.0;
    prevGyroError = 0.0;
  }

  public void setVoltage(Double leftVoltage, Double rightVoltage){
    left1.set(leftVoltage);
    left2.set(leftVoltage);
    left3.set(leftVoltage);
    
    right1.set(rightVoltage);
    right2.set(rightVoltage);
    right3.set(rightVoltage);
    return;
  }

  public void setVoltage (DriveSignal signal){
    if (this.driveControlState != DriveControlState.VOLTAGE_CONTROL) {
      driveControlState = DriveControlState.VOLTAGE_CONTROL;
      System.out.println("enter voltage control mode");
    }
    setVoltage(signal.getLeft(),signal.getRight());
  }

  //we will not need this if we have all spark motors
  public void talonFollow(){
    double leftPwm = left1.get();
    left2.set(leftPwm);
    left3.set(leftPwm);

    double rightPwm = right1.get();
    left2.set(rightPwm);
    left2.set(rightPwm);
  }

  public void setBrakeMode(boolean isBrake) {
    IdleMode sparkMode = isBrake? IdleMode.kBrake : IdleMode.kCoast;
    left1.setIdleMode(sparkMode);
    right1.setIdleMode(sparkMode);
    
    NeutralMode talonMode = isBrake? NeutralMode.Brake : NeutralMode.Coast;
    right2.setNeutralMode(talonMode);
    right3.setNeutralMode(talonMode);
    left2.setNeutralMode(talonMode);
    left3.setNeutralMode(talonMode);
  }

  public double getGyro(){
    double [] xyz_deg = new double[3];
    IMU.getAccumGyro(xyz_deg);

    currGyro = xyz_deg[2] - gyroZero;
    currGyro%=360;

    return currGyro;
  }

  public double setGyroZero(){
    double [] xyz_deg = new double[3];
    IMU.getAccumGyro(xyz_deg);

    gyroZero = xyz_deg[2];
    gyroZero%=360;

    return gyroZero;
  }

  public double getLeftEncoder(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoder(){
    return rightEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO: put networkTable logs

    getGyro();
    
    if(this.driveControlState == DriveControlState.POSITION_CONTROL){
      turn();
    }
  }

  public void setVelocity(final double leftMPS, final double rightMPS) {
    // double LActual = leftEncoder.getVelocity() / Constants.RPMpMPS;
    // System.out.printf("L want %7.2f get %7.2f err %7.2f\n", leftMPS, LActual, leftMPS - LActual);
    // System.out.printf("%f %f\n", leftMPS, rightMPS);
    if (this.driveControlState != DriveControlState.VELOCITY_CONTROL) {
      driveControlState = DriveControlState.VELOCITY_CONTROL;
      System.out.println("enter velocity control mode");
    }

    leftController.setReference(
      leftMPS * Constants.RPMpMPS,
      ControlType.kVelocity, 
      0, 
      Constants.ks * Math.signum(leftMPS) + Constants.kv * leftMPS
    );
    rightController.setReference(
      rightMPS * Constants.RPMpMPS,
      ControlType.kVelocity, 
      0, 
      Constants.ks * Math.signum(rightMPS) + Constants.kv * rightMPS
    );
  }

  private void turn(){
    this.driveControlState = DriveControlState.POSITION_CONTROL;

    double gyroError = tarGyro - currGyro; // Error = Target - Actual
    gyroError %=360;

    // System.out.println("error:" + gyroError);

    double pid = Util.pid(gyroError, 0.0, prevGyroError, timer.getDT(), 0.9, 0.0, 0.11); // prev: 1, 0, 0.016

    double leftVoltage = Util.mapValue(-20, 20, 0.3, -0.3, pid);
    double rightVoltage = Util.mapValue(-20, 20, -0.3, 0.3, pid);

    // System.out.println("Left Speed: " + leftSpeed + " Right Speed: " + rightSpeed);
    // System.out.println("Error: " + gyroError);
    
    prevGyroError = gyroError;

    this.setVoltage(leftVoltage, rightVoltage);
    // System.out.println("Turn stoped");
  }

  public void setTurn(double tar){
    if (this.driveControlState != DriveControlState.POSITION_CONTROL) {
      driveControlState = DriveControlState.POSITION_CONTROL;
      System.out.println("enter position control mode");
    }
    tarGyro = currGyro + tar;
  }

  private void setSpark(CANSparkMax spark, boolean inverted) {
    spark.restoreFactoryDefaults();
    // spark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    spark.setInverted(inverted);
  }

  private void setTalon(WPI_TalonSRX talon, boolean inverted){
    talon.setInverted(inverted);
    // left2.setNeutralMode(NeutralMode.Brake);;
  }

  private void setControler(SparkMaxPIDController controller){
    controller.setP(Constants.DRIVETRAIN_VELOCITY_GAINS.kP);
    controller.setI(0);
    controller.setFF(Constants.DRIVETRAIN_VELOCITY_GAINS.kF);
    controller.setD(Constants.DRIVETRAIN_VELOCITY_GAINS.kD);
    controller.setOutputRange(-1, 1);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
