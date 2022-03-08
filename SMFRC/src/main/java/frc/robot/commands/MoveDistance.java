// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;

   private PIDController linearPIDController = new PIDController(Constants.DRIVETRAIN_DISTANCE_GAINS.kP,
   Constants.DRIVETRAIN_DISTANCE_GAINS.kI, Constants.DRIVETRAIN_DISTANCE_GAINS.kD);

  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem DriveSubsystem
   * @param goalCamera GoalCamera
   */

  double distance;
  double linearSetpoint;
  
  public MoveDistance(Double distance, DriveSubsystem driveSubsystem) {
    this.distance = distance;
    drive = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linearPIDController.reset();

    linearSetpoint = drive.getAvgEncoderDistance() + distance;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftFF = Constants.Drive_ks;
    double rightFF = Constants.Drive_ks; 

    double linearPID = linearPIDController.calculate(drive.getAvgEncoderDistance(), linearSetpoint);

    drive.setVelocity(leftFF * Math.signum(linearPID) + linearPID, rightFF * Math.signum(linearPID) + linearPID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(linearSetpoint - drive.getAvgEncoderDistance()) <= 0.05){
      return true;
    }
    return false;
  }
}

