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

IntakeSubsystem intake;
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

  public ShootCommand(double tarDistance, IntakeSubsystem intake, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intake = intake;

    shooter = shooterSubsystem;
    indexer = indexerSubsystem;
    
    tarRPM = shooter.getFieldCorrection();

    // distance = tarDistance;
    // tarRPM = shooter.distanceToRPM(distance);

    isFinished = false;

    state = States.RETRACT;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooter);
  }

  public ShootCommand(ShooterCamera camera, IntakeSubsystem intake, IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {

    this.intake = intake;
    shooter = shooterSubsystem;
    indexer = indexerSubsystem;
    
    distance = camera.getTarDistance();
    tarRPM = shooter.distanceToRPM(distance);

    // tarRPM = shooter.getFieldCorrection();

    isFinished = false;

    state = States.RETRACT;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shooter.setRPM(shooter.distanceToRPM(distance));

    //recount the ball before shooting
    state = States.RETRACT;

    intake.extendIntake();

    indexer.setBallCount(0);
    if(indexer.getHighIR() || indexer.getLowIR()){
      indexer.setBallCount(1);
    }

    if(indexer.getHighIR() && indexer.getLowIR()){
      indexer.setBallCount(2);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(this.state);
    if(state == States.RETRACT){
      indexer.setIndexer(-0.5);
      // shooter.setPower(-0.3);
      // if(indexer.getHighIR() == false || indexer.getEntranceIR() == true){
      if(indexer.getHighIR() == false){
        state = States.PRESPIN;
        waitTar = Timer.getFPGATimestamp() + 4;
      }
    }

    if(state == States.PRESPIN){

      indexer.setIndexer(0);
      shooter.setRPM(tarRPM);
      if(Math.abs(shooter.getMasterRPM() - tarRPM) <= 100 && Math.abs(shooter.getSlaveRPM() - tarRPM) <= 100){
        state = States.CHAMBER;
        waitTar = Timer.getFPGATimestamp() + 2;
      }
    }

    if(state == States.CHAMBER){
      indexer.setIndexer(0.7);
      if(indexer.getHighIR()){
        state = States.SHOOT;
        waitTar = Timer.getFPGATimestamp() + 2;
      }
    }

    if(Timer.getFPGATimestamp() >= waitTar || state == States.SHOOT){
      if(indexer.getHighIR() == false){
        indexer.setBallCount(indexer.getBallCount()-1);
        indexer.setIndexer(0);
        waitTar = Timer.getFPGATimestamp() + 0.5;
        state = States.WAIT;
        System.out.print("bull's eye");
      }
    }

    if(state == States.WAIT){
      if(Timer.getFPGATimestamp() >= waitTar){
        state = States.PRESPIN;
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
