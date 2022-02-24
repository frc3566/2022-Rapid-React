package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class BackEject extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  IntakeSubsystem intake;
  IndexerSubsystem indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BackEject(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
    
    intake = intakeSubsystem;
    indexer = indexerSubsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.extendIntake();
    intake.setIntake(-1);
    indexer.setIndexer(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
    indexer.setIndexer(0);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
