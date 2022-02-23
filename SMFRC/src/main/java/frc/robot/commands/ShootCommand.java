// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterCamera;
import frc.robot.subsystems.ShooterSubsystem;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

IndexerSubsystem indexer;
ShooterSubsystem shooter;

double distance;
double tarRPM;

double waitTar;

boolean isFinished;

private enum States {
  RETRACT,
  PRESPIN,
  CHAMBER,
  SHOOT,
  WAIT
}

private States state;

  /**
   * 
   *
   * @param intakeSubsystem
   * @param shooterSubsystem
   */

  public ShootCommand(double tarDistance, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {

    shooter = shooterSubsystem;
    indexer = indexerSubsystem;
    
    distance = tarDistance;
    tarRPM = shooter.distanceToRPM(distance);

    isFinished = false;

    state = States.RETRACT;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooter);
  }

  public ShootCommand(ShooterCamera camera, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {

    shooter = shooterSubsystem;
    indexer = indexerSubsystem;
    
    distance = camera.getTarDistance();
    tarRPM = shooter.distanceToRPM(distance);

    isFinished = false;

    state = States.RETRACT;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shooter.setRPM(shooter.distanceToRPM(distance));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(state == States.RETRACT){
      indexer.setIndexer(-0.2);
      if(indexer.getBallCount() == 1 && indexer.getHighIR() == false){
        state = States.PRESPIN;
      }

      if(indexer.getBallCount() == 2 && (indexer.getHighIR() == false || indexer.getEntranceIR() == true)){
        state = States.PRESPIN;
      }
    }

    if(state == States.PRESPIN){
      shooter.setRPM(tarRPM);
      if(Math.abs(shooter.getRPM() - tarRPM) <= 400){
        state = States.CHAMBER;
      }
    }

    if(state == States.CHAMBER){
      indexer.setIndexer(0.7);
      if(indexer.getHighIR()){
        state = States.SHOOT;
      }
    }

    if(state == States.SHOOT){
      if(indexer.getHighIR() == false){
        indexer.setBallCount(indexer.getBallCount()-1);
        indexer.setIndexer(0);

        waitTar = Timer.getFPGATimestamp() + 2;
        state = States.WAIT;
        System.out.print("bull's eye");
      }
    }

    if(state == States.WAIT){
      if(Timer.getFPGATimestamp() >= waitTar){
        state = States.SHOOT;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setIndexer(0);
    shooter.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(indexer.getBallCount() == 0){
      return true;
    }
    return isFinished;
  }
}
