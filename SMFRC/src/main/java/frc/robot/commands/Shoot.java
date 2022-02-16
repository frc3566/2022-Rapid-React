// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

IntakeSubsystem intake;
ShooterSubsystem shooter;

double distance;
double tarRPM;

boolean shooterReady;

double shooterWaitTar;
double indexerWaitTar;

boolean isFinished;

  /**
   * 
   *
   * @param intakeSubsystem
   * @param shooterSubsystem
   */

  public Shoot(double tarDistance, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {

    intake = intakeSubsystem;
    shooter = shooterSubsystem;
    
    distance = tarDistance;
    tarRPM = shooter.distanceToRPM(distance);

    shooterWaitTar = Timer.getFPGATimestamp() + 1.5;

    shooterReady = false;

    isFinished = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setRPM(shooter.distanceToRPM(distance));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(shooter.getRPM() - tarRPM) <= 400 && Timer.getFPGATimestamp() >= shooterWaitTar){
      shooterReady = true;
      indexerWaitTar = Timer.getFPGATimestamp() + 1.5;
      intake.setIndexer(1);
      
      if(Timer.getFPGATimestamp() >= indexerWaitTar){
        isFinished = true;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
