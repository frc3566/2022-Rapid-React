package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.AutoTrajectory;

public class AutoShoot extends SequentialCommandGroup {

    public AutoShoot(AutoInit autoInit, AutoTrajectory autoTrajectory) {
        super();

        // find goal
        // aimlock
        // anchor | shoot

        this.addCommands(autoInit);

        this.addCommands(autoTrajectory);
    }
    
}
