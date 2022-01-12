package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Coms {

    private static Coms instance;

    public static Coms getInstance() {
        if(instance==null)
            synchronized(Coms.class){
                if(instance==null)
                    instance=new Coms();
            }
        return instance;
    }

    NetworkTableInstance inst;
    NetworkTable visionNT;

    // NetworkTableEntry 

    // public Coms(){
    //     inst = NetworkTableInstance.getDefault();

    //     //vision setup
    //     visionNT = inst.getTable("Vision");

    //     SmartDashboard.add()
    // }
}
