// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.sql.Time;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurnAngle extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;

   private PIDController PIDController = new PIDController(Constants.TURNING_GAINS.kP,
   Constants.TURNING_GAINS.kI, Constants.TURNING_GAINS.kD);

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem DriveSubsystem
   * @param goalCamera GoalCamera
   */

  double angle;
  double setpoint;
  
  public TurnAngle(Double angle, DriveSubsystem driveSubsystem) {
    this.angle = angle;
    drive = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PIDController.reset();

    setpoint = drive.getHeading() + angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftFF = Constants.Drive_ks;
    double rightFF = Constants.Drive_ks; 

    double angularPID = PIDController.calculate(drive.getHeading(), setpoint);

    drive.setVelocity(-(leftFF * Math.signum(angularPID) + angularPID), rightFF * Math.signum(angularPID) + angularPID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(setpoint - drive.getAvgEncoderDistance()) <= 1){
      return true;
    }
    return false;
  }
}

