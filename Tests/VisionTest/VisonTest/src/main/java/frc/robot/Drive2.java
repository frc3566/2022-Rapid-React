package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Drive2 {
  
  public static CANSparkMax left1;
  public static WPI_TalonSRX left2;
  public static WPI_TalonSRX left3;

  public static CANSparkMax right1;
  public static WPI_TalonSRX right2;
  public static WPI_TalonSRX right3;

  public static Joystick JS;

  public static CANEncoder leftE;
  public static CANEncoder rightE;

  public boolean finished;

  public double leftZero;
  public double rightZero;
  public double leftCPM; //CPM for click per meter;
  public double rightCPM;
  public double leftTar;
  public double rightTar;

  public Double leftInt;
  public Double rightInt;
  public Double leftPrevError;
  public Double rightPrevError;

  public Double prevTime;

  public Timer timer;

  public JoystickButton eStopTriger;
  public JoystickButton startButton;

  private static Drive2 instance;
  public static Drive2 getInstance() {
      if(instance==null)
          synchronized(Drive2.class){
              if(instance==null)
                  instance=new Drive2();
          }
      return instance;
  }

  public Drive2(){
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

    
    JS = new Joystick(0);
    eStopTriger = new JoystickButton(JS, 1);
    startButton = new JoystickButton(JS, 2);

    IMU = new PigeonIMU(right2);

    rightE = right1.getEncoder();
    leftE = left1.getEncoder();

    leftCPM = 21.4032; //prev: 20.58
    rightCPM = 22.61056; //prev: 20.188

    prevTime=0.0;
    leftPrevError = 0.0;
    rightPrevError = 0.0;

    timer = new Timer();
    timer.start();
  }

  public void RC(){
    double X = -0.5*JS.getRawAxis(1);
    double Y = 0.5*JS.getRawAxis(0);

    System.out.println(X);
    System.out.println(Y);

    double leftSpeed;
    double rightSpeed;
    
    leftSpeed = X + Y;
    rightSpeed = X - Y;

    go(leftSpeed, rightSpeed);
  }

  public void go(Double leftSpeed, Double rightSpeed ){

    left1.set(leftSpeed);
    left2.set(leftSpeed);
    left3.set(leftSpeed);

    right1.set(rightSpeed);
    right2.set(rightSpeed);
    right3.set(rightSpeed);
  }

  public void encoderSetZero(){
    leftZero = leftE.getPosition();
    rightZero = rightE.getPosition();
  }

  public void setMove(double leftMeter, double rightMeter){
    encoderSetZero();
    leftTar = leftZero + leftMeter*leftCPM;
    rightTar = rightZero + rightMeter*rightCPM;
    leftInt=0.0;
    rightInt=0.0;
  }

  public void move(boolean prevFinished){
    if(!prevFinished) return;
    if(finished) return;
    double leftError = leftTar - leftE.getPosition(); // Error = Target - Actual
    double rightError = rightTar - rightE.getPosition();
    System.out.println("Left Error: " + leftError + " Right Error: " + rightError);

    double t = timer.get()-prevTime;
    prevTime = timer.get();

    double pidL = computePID(leftError, leftInt, leftPrevError, t, 1.0, 0.0, 0.024); //prev: 1, 0, 0.28
    double pidR = computePID(rightError, rightInt, rightPrevError, t, 1.0, 0.0, 0.014); // prev: 1, 0, 0.016
    System.out.println("Left PID " + pidL + " Right pidR: " + pidR);

    double leftSpeed = mapValue(-20, 20, -0.7, 0.7, pidL);
    double rightSpeed = mapValue(-20, 20, -0.7, 0.7, pidR);
    System.out.println("Left Speed: " + leftSpeed + " Right Speed: " + rightSpeed);

    leftPrevError = leftError;
    rightPrevError = rightError;

    go(leftSpeed, rightSpeed);

    if(leftError + rightError < 0.7) finished = true;
  }
  
  public PigeonIMU IMU;

  public Vision vision = Vision.getInstance();

  static Drive2 turn;

  public void setTrack(){
    setTurn(0);
    if( turn == null) turn = new Drive2();
    turn.setTurn(0);
  }



  public void track() {
    double error = vision.getTar() * 0.9;

    System.out.println("Track: " + error);

    if(error != 0 && Math.abs(error) >1.8){
      turn.stop(true);
      turn = new Drive2();
      turn.setTurn(error);
      System.out.println("setTurn");
      turn.stop(true);
    }

    turn.turn(true);
    // turn.stop(turn.finished);

    System.out.println(turn.finished);

  }

  public double gyroZero;
  public double gyroTar;
  public double prevGyro;
  public double gyroInt;
  public double gyroPrevError;

  public void setTurn(double angle){
    double [] xyz_deg = new double[3];
    IMU.getAccumGyro(xyz_deg);
    gyroZero = xyz_deg[2];
    // System.out.println(gyroZero);
    gyroTar = gyroZero + angle;

    gyroInt = 0;
    gyroPrevError = 0;

    prevTime = 0.0;
    finished = false;
  }

  public void turn(boolean prevFinished){

    if(!prevFinished) return;
    if(finished) return;
    System.out.println("turn");
    double [] xyz_deg = new double[3];
    IMU.getAccumGyro(xyz_deg);

    double gyro = xyz_deg[2];

    // System.out.println(gyroTar);
    // System.out.println(gyro);
    System.out.println(xyz_deg[0] + " " + xyz_deg[1] + " " + xyz_deg[2]);

    double gyroError = gyroTar - gyro; // Error = Target - Actual

    // System.out.println(timer.get());

    double t = timer.get()-prevTime;
    prevTime = timer.get();

    double pid = computePID(gyroError, gyroInt, gyroPrevError, t, 0.9, 0.0, 0.11); // prev: 1, 0, 0.016

    System.out.println("pid: " + pid);
    // double leftSpeed = mapValue(-1, 1, 1, -1, pid);
    // double rightSpeed = mapValue(-1, 1, -1, 1, pid);
    double leftSpeed = mapValue(-20, 20, 0.3, -0.3, pid);
    double rightSpeed = mapValue(-20, 20, -0.3, 0.3, pid);
    System.out.println("Left Speed: " + leftSpeed + " Right Speed: " + rightSpeed);
    System.out.println("Error: " + gyroError);
    gyroPrevError = gyroError;

    go(leftSpeed, rightSpeed);

    if(Math.abs(gyroError)  < 2) finished = true;
    // System.out.println("Turn stoped");
  }

  public double computePID(double error, Double integral, Double prevError, Double t, Double P, Double I, Double D){
    //integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    // I doubt intergral is going to work since we are only working on one side.
    double derivative = (error - prevError) / t;
    // System.out.println(derivative);
    // System.out.println(t);
    // double derivative =0;
    double outPut = P*error + I*integral + D*derivative;
    return outPut;
  }

  public void stop(boolean started){
    System.out.println("stoping");
    if(started){
      go(0.0,0.0);
    }
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

  private double mapValue(double inLow, double inHigh, double outLow, double outHigh, double ip){
    double n = ip;
    if(n>inHigh){
      n = inHigh;
    }else if(n < inLow){
      n = inLow;
    }

    n = n - inLow;
    n = n / (inHigh-inLow);
    n = n * (outHigh-outLow);
    n = n + outLow;

    return n;
  }

}