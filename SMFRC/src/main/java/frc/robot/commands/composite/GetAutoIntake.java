package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.GoToBall;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Move;
import frc.robot.commands.getAutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeCamera;
import frc.robot.subsystems.IntakeSubsystem;

public class GetAutoIntake{

    private Command command;

    // intake | go to ball
    //        | move over ball

    public GetAutoIntake(DriveSubsystem drive, IntakeSubsystem intake, IndexerSubsystem indexer, IntakeCamera camera) {
        IntakeCommand intakeCommand = new IntakeCommand(intake, indexer);
        GoToBall goToBall = new GoToBall(drive, camera);
        Move move = new Move(-0.7, -0.7, 1.5, drive);
        
        command = intakeCommand.raceWith(goToBall.andThen(move)); 
        // command = goToBall;
    }

    public Command getCommand(){
        return command;
    }
    
}
