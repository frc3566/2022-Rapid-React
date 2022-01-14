// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GoalCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AimGoal extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;
  private GoalCamera camera;

  private TurnDrivetrain turn;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimGoal(DriveSubsystem driveSubsystem, GoalCamera goalCamera, TurnDrivetrain turnDrivetrain) {
    drive = driveSubsystem;
    camera = goalCamera;

    turn = turnDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  double tar;
  double gyroPrevError;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(camera.hasUpdated()){
      tar = camera.getTar();
      turn.setTar(tar);
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
