// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GoalCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AimLock extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;
  private GoalCamera camera;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimLock(DriveSubsystem driveSubsystem, GoalCamera goalCamera) {
    drive = driveSubsystem;
    camera = goalCamera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.reset();
  }

  double tar;
  double gyroPrevError;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(camera.isUpdated()){
      tar = camera.getTar();
      drive.setTurn(tar);
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