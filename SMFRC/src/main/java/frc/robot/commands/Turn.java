// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Turn extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;
  private PIDController pidController = new PIDController(Constants.TURNING_GAINS.kP,
   Constants.TURNING_GAINS.kI, Constants.TURNING_GAINS.kD);

  private double prevUpdateTime = 0;
  private double setpoint;
  private double tarAngle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem DriveSubsystem
   * @param goalCamera GoalCamera
   */
  public Turn(double angle, DriveSubsystem driveSubsystem) {
    drive = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    pidController.setTolerance(2, 5);

    tarAngle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    setpoint = drive.getHeading() + tarAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tarAngle = setpoint - drive.getHeading();

    double leftFF = Constants.Drive_ks * Math.signum(tarAngle);
    double rightFF = Constants.Drive_ks * Math.signum(tarAngle); 

    double pid = pidController.calculate(drive.getHeading(), setpoint);

    MathUtil.clamp(pid, 0, Constants.kMaxSpeed_Drive);

    drive.setVelocity(- leftFF - pid, rightFF + pid);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(setpoint - drive.getHeading()) <= 5){
      return true;
    }
    return false;
  }
}
