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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.DriveSignal;
import edu.wpi.first.math.util.Units;

public class DriveSubsystem extends SubsystemBase {
  
  private CANSparkMax left1;
  private WPI_TalonSRX left2, left3;

  private CANSparkMax right1;
  private WPI_TalonSRX right2, right3;

  private RelativeEncoder leftEncoder, rightEncoder;

  private double leftEncoderZero, rightEncoderZero;
  
  private SparkMaxPIDController leftController, rightController;

  private PigeonIMU gyro;

  private double gyroZero;

  private final DifferentialDriveOdometry odometry;

  public enum DriveControlState {
    VOLTAGE_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
  }

  private DriveControlState driveControlState = DriveControlState.POSITION_CONTROL;

  public DriveSubsystem(TimerSubsystem timerSubsystem) {

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
    rightEncoder.setPositionConversionFactor(Constants.ENCODER_UNITpMETER);
    rightEncoder.setVelocityConversionFactor(Constants.ENCODER_UNITpMETER);

    leftEncoder = left1.getEncoder();
    rightEncoder.setPositionConversionFactor(Constants.ENCODER_UNITpMETER);
    rightEncoder.setVelocityConversionFactor(Constants.ENCODER_UNITpMETER);

    gyro = new PigeonIMU(right2);

    resetGyro();
    getHeading();

    odometry = new DifferentialDriveOdometry(this.getRotation2d());
  }

  //moters
  public void setPower(Double leftVoltage, Double rightVoltage){
    left1.set(leftVoltage);
    left2.set(leftVoltage);
    left3.set(leftVoltage);
    
    right1.set(rightVoltage);
    right2.set(rightVoltage);
    right3.set(rightVoltage);
    return;
  }

  public void setPower (DriveSignal signal){
    if (this.driveControlState != DriveControlState.VOLTAGE_CONTROL) {
      driveControlState = DriveControlState.VOLTAGE_CONTROL;
      System.out.println("enter voltage control mode");
    }
    setPower(signal.getLeft(),signal.getRight());
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

  public void setBrake(boolean isBrake) {
    IdleMode sparkMode = isBrake? IdleMode.kBrake : IdleMode.kCoast;
    left1.setIdleMode(sparkMode);
    right1.setIdleMode(sparkMode);
    
    NeutralMode talonMode = isBrake? NeutralMode.Brake : NeutralMode.Coast;
    right2.setNeutralMode(talonMode);
    right3.setNeutralMode(talonMode);
    left2.setNeutralMode(talonMode);
    left3.setNeutralMode(talonMode);
  }

  public double getLeftEncoderDistance(){
    return leftEncoder.getPosition() - rightEncoderZero;
  }

  public double getRightEncoderDistance(){
    return rightEncoder.getPosition() - leftEncoderZero;
  }

  public void resetEncoders(){
    leftEncoderZero = leftEncoder.getPosition();
    rightEncoderZero = rightEncoder.getPosition();
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  //gyro
  public double getHeading(){
    double[] xyz_deg = new double[3];
    gyro.getAccumGyro(xyz_deg);

    double currGyro = xyz_deg[2] - gyroZero;

    return currGyro;
  }

  public double getRurnRate(){
    double[] xyz_dps = new double[3];
    gyro.getRawGyro(xyz_dps);

    return xyz_dps[2];
  }

  public Rotation2d getRotation2d(){
    return new Rotation2d(Units.degreesToRadians(getHeading()));
  }

  public double resetGyro(){
    double [] xyz_deg = new double[3];
    gyro.getAccumGyro(xyz_deg);

    gyroZero = xyz_deg[2];

    return gyroZero;
  }

  //odometry
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, this.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO: put networkTable logs

    getHeading();

    odometry.update(this.getRotation2d(), this.getLeftEncoderDistance(),
     this.getRightEncoderDistance());
    
  }

  // private void turn(){
  //   this.driveControlState = DriveControlState.POSITION_CONTROL;

  //   double gyroError = tarGyro - currGyro; // Error = Target - Actual
  //   gyroError %=360;

  //   // System.out.println("error:" + gyroError);

  //   double pid = Util.pid(gyroError, 0.0, prevGyroError, timer.getDT(), 0.9, 0.0, 0.11); // prev: 1, 0, 0.016

  //   double leftVoltage = Util.mapValue(-20, 20, 0.3, -0.3, pid);
  //   double rightVoltage = Util.mapValue(-20, 20, -0.3, 0.3, pid);

  //   // System.out.println("Left Speed: " + leftSpeed + " Right Speed: " + rightSpeed);
  //   // System.out.println("Error: " + gyroError);
    
  //   prevGyroError = gyroError;

  //   this.setPower(leftVoltage, rightVoltage);
  //   // System.out.println("Turn stoped");
  // }

  // public void setTurn(double tar){
  //   if (this.driveControlState != DriveControlState.POSITION_CONTROL) {
  //     driveControlState = DriveControlState.POSITION_CONTROL;
  //     System.out.println("enter position control mode");
  //   }
  //   tarGyro = currGyro + tar;
  // }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
