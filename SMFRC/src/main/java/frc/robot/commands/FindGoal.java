// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterCamera;
import frc.robot.subsystems.ShooterCamera.LastSeen;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FindGoal extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  DriveSubsystem drive;

  ShooterCamera camera;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FindGoal(DriveSubsystem driveSubsystem, ShooterCamera shooterCamera) {
    
    drive = driveSubsystem;

    camera = shooterCamera;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(camera.lastSeen == LastSeen.LEFT){
      drive.setVelocity(-0.3, 0.3);
    }else{
      drive.setVelocity(0.3, -0.3);
    }
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
    return camera.goalDetected();
  }
}
