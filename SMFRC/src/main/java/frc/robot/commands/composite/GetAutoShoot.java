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

    public GetAutoShoot(DriveSubsystem drive, IndexerSubsystem indexer, ShooterSubsystem shooter, ShooterCamera camera) {
        // find goal
        // aimlock
        // anchor | shoot

        FindGoal findGoal = new FindGoal(drive, camera);
        AimLock aimLock = new AimLock(drive, camera);
        Anchor anchor = new Anchor(drive);

        ShootCommand shoot = new ShootCommand(camera, indexer, shooter);

        command = aimLock.andThen(anchor.raceWith(shoot));
    }

    public Command getCommand(){
        return command;
    }
    
}
