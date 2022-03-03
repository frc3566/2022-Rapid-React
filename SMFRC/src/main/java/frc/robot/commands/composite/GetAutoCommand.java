package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Move;
import frc.robot.commands.MoveDistance;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurnAngle;
import frc.robot.commands.getAutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeCamera;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GetAutoCommand extends SequentialCommandGroup {

    // init
    // trajectroy
    // intake
    // turn to goal
    // aim lock
    // anchor | shoot

    public GetAutoCommand(DriveSubsystem drive, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, AutoInit autoInit, GetAutoIntake getAutoIntake, GetAutoShoot getAutoShoot) {
        super();
        
        IntakeCommand intakeCommand = new IntakeCommand(2, intake, indexer);
        MoveDistance move = new MoveDistance(1.0, drive);
        TurnAngle turn = new TurnAngle(90.0, drive);

        // 2 ball basic
        this.addCommands(autoInit);
        this.addCommands(new ShootCommand(2, intake, indexer, shooter));
        this.addCommands((new Move(-0.7, -0.7, 2.5, drive).alongWith(new IntakeCommand(2, intake, indexer)).withTimeout(6)));
        this.addCommands(new ShootCommand(3, intake, indexer, shooter));

        // 2 ball pid
        // this.addCommands(autoInit);
        // this.addCommands(new ShootCommand(2, intake, indexer, shooter));
        // this.addCommands((new MoveDistance(-1.5, drive).alongWith(new IntakeCommand(2, intake, indexer)).withTimeout(6)));
        // this.addCommands(new ShootCommand(3, intake, indexer, shooter));
        // this.addCommands(new TurnAngle(90.0, drive));

        // this.addCommands(getAutoIntake.getCommand());
        // this.addCommands(new Move(-0.5, -0.5, 1.0, drive));
        // this.addCommands(autoShoot);
        // this.addCommands(getAutoTrajectory.getTrajectory(drive)); 

    }
    
}
