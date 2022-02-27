// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param intakeSubsystem
   * @param indexerSubsystem
   */

   IntakeSubsystem intake;
   IndexerSubsystem indexer;

   int targetBallcnt;

  public IntakeCommand(int tar, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {

    intake = intakeSubsystem;
    indexer = indexerSubsystem;

    targetBallcnt = tar;

    if(targetBallcnt > 2){
      targetBallcnt = 2;
    }else if (targetBallcnt < 0){
      targetBallcnt = 0;
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.extendIntake();
    intake.setIntake(0.9);
    indexer.setIndexer(0.9);

    //recount the ball before shooting
    indexer.setBallCount(0);
    if(indexer.getHighIR()){
      indexer.setBallCount(indexer.getBallCount()+1);
    }
    if(indexer.getLowIR()){
      indexer.setBallCount(indexer.getBallCount()+1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(indexer.getBallCount() == 0 && indexer.getHighIR()){
        indexer.setBallCount(1);
      }

      if(indexer.getBallCount() == 1 && indexer.getLowIR()){
        indexer.setBallCount(2);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      intake.setIntake(0);
      intake.contractIntake();
      indexer.setIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(indexer.getBallCount() >= targetBallcnt){
        return true;
    }
    return false;
  }
}
