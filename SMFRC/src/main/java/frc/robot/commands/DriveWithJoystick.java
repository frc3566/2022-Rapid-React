// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.DriveSignal;
import frc.robot.util.MathUtil;

/** An example command that uses an example subsystem. */
public class DriveWithJoystick extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;

  private Joystick JS;
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem DriveSubsystem
   */
  public DriveWithJoystick(Joystick joystick, DriveSubsystem driveSubsystem) {
    drive = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    JS = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = getForwardThrottle();
    boolean isQuickTurn = isQuickTurn();
    
    speed = speed * speed * Math.signum(speed);
    if (!isOverride())  // override gives full power for hitting
      speed *= 0.8;

    double rotation_sign = (isQuickTurn || speed>0? 1:-1) ;
    double rotation = getRotationThrottle();
    rotation *= 0.7;

    if (isReversed())speed *= -1;

    DriveSignal signal = cheesyDrive(
      speed,
      rotation * rotation_sign, 
      isQuickTurn, 
      false
    );

    drive.setPower(signal);

    // drive.setPower(0.5,0.5);
    // System.out.println(signal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

   // drivetrain

  public boolean isOverride() {
    return JS.getRawButton(2);
  }
  public boolean isReversed() {
    return getSlider() < 0.5;
  }

  public boolean isQuickTurn() {
    return JS.getRawButton(3);
  }

  public double getForwardThrottle() {
    return JS.getRawAxis(0) * -1;
  }

  public double getRotationThrottle() {
    return JS.getRawAxis(1);
  }

  public double getSlider() {
    return (1 - JS.getRawAxis(3)) / 2;
  }

  private static final double kThrottleDeadband = 0.035;
  private static final double kWheelDeadband = 0.02;

  // These factor determine how fast the wheel traverses the "non linear" sine curve.

  private static final double kLowNegInertiaThreshold = 0.65;
  private static final double kLowNegInertiaTurnScalar = 3.0;
  private static final double kLowNegInertiaCloseScalar = 3.0;
  private static final double kLowNegInertiaFarScalar = 4.0;

  private static final double kWheelQuckTurnScalar = .65;

  private static final double kQuickStopDeadband = 0.5;
  private static final double kQuickStopWeight = 0.125;
  private static final double kQuickStopScalar = 2.8;

  private double mOldWheel = 0.0;
  private double mQuickStopAccumlator = 0.0;
  private double mNegInertiaAccumlator = 0.0;

  private DriveSignal cheesyDrive(double throttle, double wheel,
   boolean isQuickTurn, boolean isHighGear) {

  wheel = MathUtil.deadband(wheel, kWheelDeadband);
  throttle = MathUtil.deadband(throttle, kThrottleDeadband);

  double negInertia = wheel - mOldWheel;
  mOldWheel = wheel;

  double leftPwm, rightPwm, overPower;
  double sensitivity;

  double angularPower;
  double linearPower;

  // NOTE nolinearity has been removed
  // Negative inertia!
  double negInertiaScalar;
  if (wheel * negInertia > 0) {
  // If we are moving away from 0.0, aka, trying to get more wheel.
  negInertiaScalar = kLowNegInertiaTurnScalar;
  } else {
  // Otherwise, we are attempting to go back to 0.0.
  if (Math.abs(wheel) > kLowNegInertiaThreshold) {
  negInertiaScalar = kLowNegInertiaFarScalar;
  } else {
  negInertiaScalar = kLowNegInertiaCloseScalar;
  }
  }
  sensitivity = 1;
  double negInertiaPower = negInertia * negInertiaScalar;
  mNegInertiaAccumlator += negInertiaPower;

  wheel = wheel + mNegInertiaAccumlator;
  if (mNegInertiaAccumlator > 1) {
  mNegInertiaAccumlator -= 1;
  } else if (mNegInertiaAccumlator < -1) {
  mNegInertiaAccumlator += 1;
  } else {
  mNegInertiaAccumlator = 0;
  }
  linearPower = throttle;

  // Quickturn!
  if (isQuickTurn) {
  if (Math.abs(linearPower) < kQuickStopDeadband) {
  double alpha = kQuickStopWeight;
  mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
  + alpha * Math.abs(throttle)* kQuickStopScalar;
  }
  overPower = 1.0;
  angularPower = wheel * kWheelQuckTurnScalar;
  } else {
  overPower = 0.0;
  angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
  if (mQuickStopAccumlator > 1) {
  mQuickStopAccumlator -= 1;
  } else if (mQuickStopAccumlator < -1) {
  mQuickStopAccumlator += 1;
  } else {
  mQuickStopAccumlator = 0.0;
  }
  }

  rightPwm = leftPwm = linearPower;
  leftPwm += angularPower;
  rightPwm -= angularPower;

  if (leftPwm > 1.0) {
  rightPwm -= overPower * (leftPwm - 1.0);
  leftPwm = 1.0;
  } else if (rightPwm > 1.0) {
  leftPwm -= overPower * (rightPwm - 1.0);
  rightPwm = 1.0;
  } else if (leftPwm < -1.0) {
  rightPwm += overPower * (-1.0 - leftPwm);
  leftPwm = -1.0;
  } else if (rightPwm < -1.0) {
  leftPwm += overPower * (-1.0 - rightPwm);
  rightPwm = -1.0;
  }
  return new DriveSignal(leftPwm, rightPwm);
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
