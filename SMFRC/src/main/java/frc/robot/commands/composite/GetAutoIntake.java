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
import frc.robot.subsystems.IntakeSubsystem;

public class GetAutoIntake{

    private Command command;

    public GetAutoIntake(DriveSubsystem drive, IntakeCommand intake, GoToBall goToBall) {

        // intake | go to ball
        //        | move over ball
        Move move = new Move(1.0, 1.0, 1.0, drive);
        
        command = intake.alongWith(goToBall.andThen(move));     

    }

    public GetAutoIntake(DriveSubsystem drive, IntakeSubsystem intake, IndexerSubsystem indexer, GoToBall goToBall) {
    }

    public Command getCommand(){
        return command;
    }
    
}
