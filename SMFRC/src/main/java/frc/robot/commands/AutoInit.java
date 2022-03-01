// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoInit extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param 
   */
  ClimberSubsystem climer;
  DriveSubsystem drive;
  IntakeSubsystem intake;
  ShooterSubsystem shooter;

  boolean finished = false;

  public AutoInit(ClimberSubsystem climerSubsystem, DriveSubsystem driveSubsystem, 
  IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {

    climer = climerSubsystem;
    drive = driveSubsystem;
    intake = intakeSubsystem;
    shooter = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climer, drive, intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climer.contract();
    drive.resetGyro();
    drive.resetEncoders();
    drive.resetOdometry(new Pose2d());
    drive.setBrake(true);
    intake.extendIntake();
    shooter.setRPM(2000);
    finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
