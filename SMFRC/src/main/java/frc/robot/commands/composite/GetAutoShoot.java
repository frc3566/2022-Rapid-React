package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AimLock;
import frc.robot.commands.Anchor;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterCamera;
import frc.robot.subsystems.ShooterSubsystem;

public class GetAutoShoot{

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;
    private ShooterCamera camera;

    // find goal
    // aimlock
    // anchor | shoot

    public GetAutoShoot(DriveSubsystem drive, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, ShooterCamera camera) {
        this.drive = drive;
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.camera = camera;
    }

    public Command getCommand(){
        Command command;
        // FindGoal findGoal = new FindGoal(drive, camera);
        AimLock aimLock = new AimLock(drive, camera);
        // Anchor anchor = new Anchor(drive);

        ShootCommand shoot = new ShootCommand(camera, intake, indexer, shooter);

        command = aimLock.andThen(shoot);
        return command;
    }
    
}
