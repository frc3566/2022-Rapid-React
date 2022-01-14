// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TimerSubsystem;

import frc.robot.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurnDrivetrain extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;
  private TimerSubsystem timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public TurnDrivetrain(double targetAngleDegrees, DriveSubsystem driveSubsystem, TimerSubsystem timerSubsystem) {
    drive = driveSubsystem;
    timer = timerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    tarAngle = targetAngleDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tarAngle = currAngle;
  }

  private double currAngle;
  private double tarAngle;
  private double prevError;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currAngle = drive.getGyro();

    double gyroError = tarAngle - currAngle; // Error = Target - Actual

    gyroError %=360;

    double pid = Util.pid(gyroError, 0.0, prevError, timer.getDT(), 0.9, 0.0, 0.11); // prev: 1, 0, 0.016

    double leftSpeed = Util.mapValue(-20, 20, 0.3, -0.3, pid);
    double rightSpeed = Util.mapValue(-20, 20, -0.3, 0.3, pid);

    // System.out.println("Left Speed: " + leftSpeed + " Right Speed: " + rightSpeed);
    // System.out.println("Error: " + gyroError);
    
    prevError = gyroError;

    drive.setSpeed(leftSpeed, rightSpeed);
    // System.out.println("Turn stoped");
  }

  public void setTar(double tar){
    tarAngle = currAngle + tar;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
