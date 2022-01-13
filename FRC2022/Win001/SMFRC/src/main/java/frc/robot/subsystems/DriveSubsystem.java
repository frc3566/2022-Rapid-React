// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
    private CANSparkMax left1;
    private WPI_TalonSRX left2;
    private WPI_TalonSRX left3;
  
    private CANSparkMax right1;
    private WPI_TalonSRX right2;
    private WPI_TalonSRX right3;
  
    private RelativeEncoder leftE;
    private RelativeEncoder rightE;

    private PigeonIMU IMU;

  public DriveSubsystem() {
    left1 = new CANSparkMax(12, MotorType.kBrushless);
    setSpark(left1, false);

    left2 = new WPI_TalonSRX(3);
    setTalon(left2, false);

    left3 = new WPI_TalonSRX(4);
    setTalon(left3, false);

    right1 = new CANSparkMax(11, MotorType.kBrushless);
    setSpark(right1, true);

    right2 = new WPI_TalonSRX(1);
    setTalon(right2, false);

    right3 = new WPI_TalonSRX(2);
    setTalon(right3, true);

    rightE = right1.getEncoder();
    leftE = left1.getEncoder();

    IMU = new PigeonIMU(right2);
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

  public void setSpeeds(Double leftSpeed, Double rightSpeed){
    left1.set(leftSpeed);
    left2.set(leftSpeed);
    left3.set(leftSpeed);

    right1.set(rightSpeed);
    right2.set(rightSpeed);
    right3.set(rightSpeed);

    return;
  }

  public double getGyro(){
    double [] xyz_deg = new double[3];
    IMU.getAccumGyro(xyz_deg);
    double gyro = xyz_deg[2];

    return gyro;
  }

  public double getLeft(){
    return leftE.getPosition();
  }

  public double getRight(){
    return rightE.getPosition();
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
