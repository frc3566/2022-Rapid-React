// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeCamera;

import java.sql.Time;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GoToBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;
  private IntakeCamera camera;

  private PIDController angularPIDController = new PIDController(Constants.TURNING_GAINS.kP,
   Constants.TURNING_GAINS.kI, Constants.TURNING_GAINS.kD);

   private PIDController linearPIDController = new PIDController(Constants.DRIVETRAIN_DISTANCE_GAINS.kP,
   Constants.DRIVETRAIN_DISTANCE_GAINS.kI, Constants.DRIVETRAIN_DISTANCE_GAINS.kD);

   private double prevUpdateTime = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem DriveSubsystem
   * @param goalCamera GoalCamera
   */

  double angularSetpoint;
  double linearSetpoint;

  double angularUpdateTime;
  double linearUpdateTime;

  boolean angularComplete;
  boolean linearComplete;

  double minLinearSpeed = 1;

  
  public GoToBall(DriveSubsystem driveSubsystem, IntakeCamera intakeCamera) {
    drive = driveSubsystem;
    camera = intakeCamera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    angularPIDController.setTolerance(0.5);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angularPIDController.reset();
    linearPIDController.reset();

    angularSetpoint = drive.getHeading() + camera.getTarAngle();
    linearSetpoint = drive.getAvgEncoderDistance() - camera.getTarDistance();

    angularUpdateTime = Timer.getFPGATimestamp();
    linearUpdateTime = Timer.getFPGATimestamp();

    angularComplete = false;
    linearComplete = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(camera.ballDetected() && Timer.getFPGATimestamp() > angularUpdateTime && angularComplete){
      angularSetpoint = drive.getHeading() + camera.getTarAngle();
      angularUpdateTime = Timer.getFPGATimestamp() + 0.6;
      angularComplete = false;
      System.out.println("update anuglar");
    }

    if(camera.ballDetected() && Timer.getFPGATimestamp() > linearUpdateTime && linearComplete){
      linearSetpoint = drive.getAvgEncoderDistance() - (camera.getTarDistance());
      linearUpdateTime = Timer.getFPGATimestamp() + 0.6;
      linearComplete = false;
      System.out.println("update linear");
    }

    // System.out.println(drive.getAvgEncoderDistance() + " " + linearSetpoint + " " + linearComplete);


    // ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(0, 0, v_angular);

    // DifferentialDriveWheelSpeeds wheelSpeeds = Constants.kDriveKinematics.toWheelSpeeds(adjustedSpeeds);

    double leftFF = Constants.Drive_ks;
    double rightFF = Constants.Drive_ks; 

    double angularPID;
    double linearPID;

    if(Math.abs(drive.getHeading() - angularSetpoint) <= 1){
      angularPID = 0;
      angularComplete = true;
    }else{
      angularPID = angularPIDController.calculate(drive.getHeading(), angularSetpoint);
    }

    if(Math.abs(drive.getAvgEncoderDistance() - linearSetpoint) <= 0.05){
      linearPID = 0;
      linearComplete = true;
      // System.out.println("linear complete");
    }else{
      linearPID = linearPIDController.calculate(drive.getAvgEncoderDistance(), linearSetpoint);
    }

    if(camera.ballDetected() && Math.abs(linearPID)< minLinearSpeed){
      linearPID = Math.signum(minLinearSpeed);
    }

    double left = linearPID + angularPID;
    double right = linearPID - angularPID;

    // double left = linearPID;
    // double right = linearPID;

    // double left = angularPID;
    // double right = - angularPID;

    if(Math.abs(left) > Constants.kMaxSpeed_Drive){
      left = Constants.kMaxSpeed_Drive * Math.signum(left);
      right = right * Constants.kMaxSpeed_Drive / Math.abs(left);
    }

    if(Math.abs(right) > Constants.kMaxSpeed_Drive){
      right = Constants.kMaxSpeed_Drive * Math.signum(right);
      left = left * Constants.kMaxSpeed_Drive / Math.abs(right);
    }

    drive.setVelocity(leftFF * Math.signum(left) + left, rightFF * Math.signum(right) + right);
    
    System.out.println(left + " " + right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(camera.getTarDistance() <= 1 && Math.abs(camera.getTarAngle()) <= 10){
      return true;
    }
    return false;
  }
}

