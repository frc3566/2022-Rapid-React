// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants;
import frc.robot.util.DriveSignal;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  
  private CANSparkMax left1;
  private CANSparkMax left2, left3;
  // private WPI_TalonSRX left2, left3;

  private CANSparkMax right1;
  private CANSparkMax right2, right3;
  // private WPI_TalonSRX right2, right3;

  private RelativeEncoder leftEncoder, rightEncoder;

  private double leftEncoderZero, rightEncoderZero;
  
  private SparkMaxPIDController leftController, rightController;

  private AHRS gyro;

  private double gyroZero;

  private DifferentialDriveOdometry odometry;

  public enum DriveControlState {
    VOLTAGE_CONTROL,
    VELOCITY_CONTROL,
    POSITION_CONTROL
  }

  private DriveControlState driveControlState = DriveControlState.POSITION_CONTROL;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable nt = inst.getTable("LiveWindow/DriveSubsystem");
  private NetworkTableEntry leftDistanceEntry = nt.getEntry("left_encoder");
  private NetworkTableEntry rightDistanceEntry = nt.getEntry("right_encoder");
  private NetworkTableEntry leftRPMEntry = nt.getEntry("left_RPM");
  private NetworkTableEntry rightRPMEntry = nt.getEntry("right_RPM");
  private NetworkTableEntry headingEntry = nt.getEntry("heading");

  public DriveSubsystem() {

    // dummy
    // left1 = new CANSparkMax(12, MotorType.kBrushless);
    // setSpark(left1, false);
    // leftController = left1.getPIDController();
    // setControler(leftController);

    // left2 = new WPI_TalonSRX(3);
    // setTalon(left2, false);

    // left3 = new WPI_TalonSRX(4);
    // setTalon(left3, false);

    // right1 = new CANSparkMax(11, MotorType.kBrushless);
    // setSpark(right1, true);
    // rightController = right1.getPIDController();
    // setControler(rightController);

    // right2 = new WPI_TalonSRX(1);
    // setTalon(right2, false);

    // right3 = new WPI_TalonSRX(2);
    // setTalon(right3, true);

    // good boy
    left1 = new CANSparkMax(1, MotorType.kBrushless);
    left1.setInverted(true);;
    leftController = left1.getPIDController();
    setControler(leftController);

    left2 = new CANSparkMax(2, MotorType.kBrushless);
    left2.follow(left1);

    left3 = new CANSparkMax(3, MotorType.kBrushless);
    left3.follow(left1);

    right1 = new CANSparkMax(4, MotorType.kBrushless);
    right1.setInverted(false);
    rightController = right1.getPIDController();
    setControler(rightController);

    right2 = new CANSparkMax(5, MotorType.kBrushless);
    right2.follow(right1);

    right3 = new CANSparkMax(6, MotorType.kBrushless);
    right3.follow(right1);

    rightEncoder = right1.getEncoder();
    rightEncoder.setPositionConversionFactor(Constants.ENCODER_UNIT2METER);
    rightEncoder.setVelocityConversionFactor(Constants.ENCODER_UNIT2METER);

    leftEncoder = left1.getEncoder();
    leftEncoder.setPositionConversionFactor(Constants.ENCODER_UNIT2METER);
    leftEncoder.setVelocityConversionFactor(Constants.ENCODER_UNIT2METER);

    leftController.setP(Constants.DRIVETRAIN_VELOCITY_GAINS.kP);
    leftController.setI(Constants.DRIVETRAIN_VELOCITY_GAINS.kI);
    leftController.setD(Constants.DRIVETRAIN_VELOCITY_GAINS.kD);

    gyro = new AHRS(SPI.Port.kMXP);

    resetGyro();
    getHeading();

    odometry = new DifferentialDriveOdometry(this.getRotation2d());

  }

  //moters
  public void setPower(Double leftPower, Double rightPower){
    left1.set(leftPower);
    left2.set(leftPower);
    left3.set(leftPower);
    
    right1.set(rightPower);
    right2.set(rightPower);
    right3.set(rightPower);
    return;
  }

  public void setPower(DriveSignal signal){
    if (this.driveControlState != DriveControlState.VOLTAGE_CONTROL) {
      driveControlState = DriveControlState.VOLTAGE_CONTROL;
      System.out.println("enter voltage control mode");
    }
    setPower(signal.getLeft(),signal.getRight());
  }

  public void setVoltage(double left, double right){
    setPower(left, right);
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

  private void setTalon(WPI_TalonSRX talon, boolean inverted){
    talon.setInverted(inverted);
    // left2.setNeutralMode(NeutralMode.Brake);;
  }

  private void setControler(SparkMaxPIDController controller){
    controller.setP(Constants.DRIVETRAIN_VELOCITY_GAINS.kP);
    controller.setI(0);
    controller.setFF(Constants.DRIVETRAIN_VELOCITY_GAINS.kFF);
    controller.setD(Constants.DRIVETRAIN_VELOCITY_GAINS.kD);
    controller.setOutputRange(-1, 1);
  }

  public void setBrake(boolean isBrake) {
    IdleMode sparkMode = isBrake? IdleMode.kBrake : IdleMode.kCoast;
    left1.setIdleMode(sparkMode);
    right1.setIdleMode(sparkMode);

    left2.setIdleMode(sparkMode);
    right2.setIdleMode(sparkMode);
    left3.setIdleMode(sparkMode);
    right3.setIdleMode(sparkMode);
    
  //   NeutralMode talonMode = isBrake? NeutralMode.Brake : NeutralMode.Coast;
  //   right2.setNeutralMode(talonMode);
  //   right3.setNeutralMode(talonMode);
  //   left2.setNeutralMode(talonMode);
  //   left3.setNeutralMode(talonMode);
  }

  public double getLeftEncoderDistance(){
    return leftEncoder.getPosition() - leftEncoderZero;
  }

  public double getRightEncoderDistance(){
    return rightEncoder.getPosition() - rightEncoderZero;
  }

  public double getAvgEncoderDistance(){
    return (getLeftEncoderDistance() + getRightEncoderDistance())/2;
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
      leftMPS,
      ControlType.kVelocity, 
      0, 
      Constants.Drive_ks * Math.signum(leftMPS) + Constants.Drive_kv * leftMPS
    );

    rightController.setReference(
      rightMPS,
      ControlType.kVelocity, 
      0, 
      Constants.Drive_ks * Math.signum(rightMPS) + Constants.Drive_kv * rightMPS
    );
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  //gyro
  public double getHeading(){
    double[] xyz_deg = new double[3];
    // gyro.getAccumGyro(xyz_deg);

    // double currGyro = xyz_deg[2] - gyroZero;

    double currGyro = gyro.getAngle() - gyroZero;

    return currGyro;
  }

  public double getRurnRate(){
    double[] xyz_dps = new double[3];
    // gyro.getRawGyro(xyz_dps);

    // return xyz_dps[2];

    return gyro.getRate();
  }

  public Rotation2d getRotation2d(){
    return new Rotation2d(Units.degreesToRadians(getHeading()));
  }

  public double resetGyro(){
    double [] xyz_deg = new double[3];
    // gyro.getAccumGyro(xyz_deg);

    // gyroZero = xyz_deg[2];

    gyroZero = gyro.getAngle();

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

  public void disabled(){
    setBrake(false);
    setPower(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    leftDistanceEntry.setDouble(getLeftEncoderDistance());
    rightDistanceEntry.setDouble(getRightEncoderDistance());
    leftRPMEntry.setDouble(leftEncoder.getVelocity());
    rightRPMEntry.setDouble(rightEncoder.getVelocity());
    headingEntry.setDouble(getHeading());

    odometry.update(this.getRotation2d(), this.getLeftEncoderDistance(),
     this.getRightEncoderDistance());

    // System.out.println("left RPM: " + leftEncoder.getVelocity());
    // System.out.println("right RPM: " + rightEncoder.getVelocity());
    // System.out.println("right RPM: " + rightEncoder.getVelocity())
    // System.out.println("heading: " + getHeading());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


}
