package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.getAutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCommand extends SequentialCommandGroup {

    public AutoCommand(DriveSubsystem drive, AutoInit autoInit) {
        super();

        // init
        // trajectroy
        // intake
        // turn to goal
        // aim lock
        // anchor | shoot

        this.addCommands(autoInit);
        this.addCommands(getAutoTrajectory.getTrajectory(drive));


    }
    
}
