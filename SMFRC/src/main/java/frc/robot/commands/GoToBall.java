// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
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
  
  public GoToBall(DriveSubsystem driveSubsystem, IntakeCamera intakeCamera) {
    drive = driveSubsystem;
    camera = intakeCamera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    angularPIDController.setTolerance(2, 5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angularSetpoint = camera.getTarAngle();
    linearSetpoint= camera.getTarDistance();
    angularPIDController.reset();
    linearPIDController.reset();
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(prevUpdateTime != camera.getLastUpdateTime()){
      angularSetpoint = drive.getHeading() + camera.getTarAngle();
      linearSetpoint = drive.getAvgEncoderDistance() + camera.getTarDistance();
      // System.out.println("update turning");
    }

    // ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(0, 0, v_angular);

    // DifferentialDriveWheelSpeeds wheelSpeeds = Constants.kDriveKinematics.toWheelSpeeds(adjustedSpeeds);

    double leftFF = Constants.Drive_ks;
    double rightFF = Constants.Drive_ks; 

    double angularPID = angularPIDController.calculate(drive.getHeading(), angularSetpoint);
    
    double linearPID = linearPIDController.calculate(drive.getAvgEncoderDistance(), linearSetpoint);

    double left = linearPID - angularPID;
    double right = linearPID + angularPID;

    if(Math.abs(left) > Constants.kMaxSpeed_Drive){
      left = Constants.kMaxSpeed_Drive;
      right = right * Constants.kMaxSpeed_Drive / left;
    }

    if(Math.abs(right) > Constants.kMaxSpeed_Drive){
      right = Constants.kMaxSpeed_Drive;
      left = left * Constants.kMaxSpeed_Drive / right;
    }

    drive.setVelocity(leftFF * Math.signum(left) + left, rightFF * Math.signum(rightFF) + right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(camera.getTarDistance() <= 0.7 && Math.abs(camera.getTarAngle()) <= 20){
      return true;
    }

    return false;
  }
}
