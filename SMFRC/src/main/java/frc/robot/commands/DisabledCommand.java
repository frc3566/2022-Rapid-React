// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DisabledCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * disables all subsystem by running the disabled method.
   *
   * @param subsystem The subsystem used by this command.
   */

   DriveSubsystem drive;
   IntakeSubsystem intake;
   IndexerSubsystem indexer;
   ShooterSubsystem shooter;

   boolean isFinished = false;

  public DisabledCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = driveSubsystem;
    intake = intakeSubsystem;
    indexer = indexerSubsystem;
    shooter = shooterSubsystem;

    addRequirements(drive, intake, indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.disabled();
    drive.setBrake(false);
    intake.disabled();
    shooter.disabled();
    indexer.disabled();

    isFinished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.disabled();
    drive.setBrake(false);
    intake.disabled();
    shooter.disabled();
    indexer.disabled();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
