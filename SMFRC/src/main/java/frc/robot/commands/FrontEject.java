package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FrontEject extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  IndexerSubsystem indexer;
  ShooterSubsystem shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FrontEject(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem) {
    
    indexer = indexerSubsystem;
    shooter = shooterSubsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setIndexer(0.7);
    // shooter.setRPM(700);
    shooter.setRPM(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setIndexer(0);
    shooter.setRPM(0);
    indexer.setBallCount(0);
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
