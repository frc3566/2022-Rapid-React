// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveWithJoystick extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;

  private Joystick JS;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveWithJoystick(DriveSubsystem subsystem) {
    drive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    JS = new Joystick(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double X = -0.5*JS.getRawAxis(1);
    double Y = 0.5*JS.getRawAxis(0);

    System.out.println(X);
    System.out.println(Y);

    double leftSpeed;
    double rightSpeed;
    
    leftSpeed = X + Y;
    rightSpeed = X - Y;

    drive.setSpeed(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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

  public double[] cheesyDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear) {

  wheel = handleDeadband(wheel, kWheelDeadband);
  throttle = handleDeadband(throttle, kThrottleDeadband);

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
  + alpha * Util.limit(wheel, 1.0) * kQuickStopScalar;
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
  return new double[2]{leftPwm, rightPwm};
}

public double handleDeadband(double val, double deadband) {
  return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
