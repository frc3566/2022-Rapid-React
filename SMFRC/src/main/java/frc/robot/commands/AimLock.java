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
public class AimLock extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;
  private ShooterCamera camera;

  private PIDController angularPID = new PIDController(Constants.TURNING_GAINS.kP,
   Constants.TURNING_GAINS.kI, Constants.TURNING_GAINS.kD);

   private PIDController distancePID = new PIDController(Constants.TURNING_GAINS.kP,
   Constants.TURNING_GAINS.kI, Constants.TURNING_GAINS.kD);

   private double prevUpdateTime = 0;
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem DriveSubsystem
   * @param goalCamera GoalCamera
   */
  public AimLock(DriveSubsystem driveSubsystem, ShooterCamera goalCamera) {
    drive = driveSubsystem;
    camera = goalCamera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    angularPID.setTolerance(2, 5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angularPID.reset();
  }

  double tar;
  double gyroPrevError;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(prevUpdateTime != camera.getLastUpdateTime()){

      double setPoint = drive.getHeading() + camera.getTarAngle();

      double angularFeedForward = Constants.Drive_ks * Math.signum(setPoint);

      double power = MathUtil.clamp(angularPID.calculate(drive.getHeading(), setPoint),-0.3,0.3);

      drive.setPower(-power, power);
      // System.out.println("update turning");
    }
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
