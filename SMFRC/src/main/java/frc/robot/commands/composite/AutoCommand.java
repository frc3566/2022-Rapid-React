package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.getAutoTrajectory;

public class AutoCommand extends SequentialCommandGroup {

    public AutoCommand(AutoInit autoInit, getAutoTrajectory autoTrajectory) {
        super();

        // init
        // trajectroy
        // intake
        // turn to goal
        // aim lock
        // anchor | shoot

        this.addCommands(autoInit);

    }
    
}
