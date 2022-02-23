package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimLock;
import frc.robot.commands.Anchor;
import frc.robot.commands.AutoInit;
import frc.robot.commands.FindGoal;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.getAutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterCamera;
import frc.robot.subsystems.ShooterSubsystem;

public class GetAutoShoot{

    private Command command;

    public GetAutoShoot(FindGoal findGoal, AimLock aimLock, Anchor anchor, ShootCommand shoot) {
        // find goal
        // aimlock
        // anchor | shoot

        command = findGoal.andThen(aimLock).andThen(anchor.alongWith(shoot));
    }

    public GetAutoShoot(FindGoal findGoal, AimLock aimLock, Anchor anchor, IndexerSubsystem indexer,
            ShooterSubsystem shooter) {
    }

    public Command getCommand(){
        return command;
    }
    
}
