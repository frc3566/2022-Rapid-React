package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Move;
import frc.robot.commands.getAutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeCamera;
import frc.robot.subsystems.IntakeSubsystem;

public class GetAutoCommand extends SequentialCommandGroup {

    // init
    // trajectroy
    // intake
    // turn to goal
    // aim lock
    // anchor | shoot

    public GetAutoCommand(DriveSubsystem drive, IntakeSubsystem intake, IndexerSubsystem indexer, AutoInit autoInit) {
        super();
        
        IntakeCommand intakeCommand = new IntakeCommand(2, intake, indexer);

        this.addCommands(autoInit);
        // this.addCommands(intakeCommand);
        // this.addCommands(new Autointake(intake, intakeCamera, );
        this.addCommands(new Move(0.5,0.5,1.0,drive));
        
        this.addCommands(getAutoTrajectory.getTrajectory(drive));


    }
    
}
