package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.AutoTrajectory;

public class AutoCommand extends SequentialCommandGroup {

    public AutoCommand(AutoInit autoInit, AutoTrajectory autoTrajectory) {
        super();
        this.addCommands(autoInit);

        this.addCommands(autoTrajectory);
    }
    
}
