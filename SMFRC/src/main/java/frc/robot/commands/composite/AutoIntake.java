package frc.robot.commands.composite;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoInit;
import frc.robot.commands.getAutoTrajectory;

public class AutoIntake extends SequentialCommandGroup {

    public AutoIntake(AutoInit autoInit, getAutoTrajectory autoTrajectory) {
        super();

        // deploy intake
        // go to ball
        // move over ball
        this.addCommands(autoInit);

    }
    
}
