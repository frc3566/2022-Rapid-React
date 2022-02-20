package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.getAutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterCamera;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends SequentialCommandGroup {

    public AutoShoot(DriveSubsystem drive, ShooterSubsystem shooter, ShooterCamera camera) {
        super();

        // find goal
        // aimlock
        // anchor | shoot


    }
    
}
