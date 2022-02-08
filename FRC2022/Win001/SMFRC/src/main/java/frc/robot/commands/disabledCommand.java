// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class disabledCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * disables all subsystem by running the disabled method.
   *
   * @param subsystem The subsystem used by this command.
   */

   DriveSubsystem drive;
   Shooter shooter;

  public disabledCommand(DriveSubsystem driveSubsystem, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, shooter);

    drive = driveSubsystem;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.disabled();
    shooter.disabled();
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
    return false;
  }
}
