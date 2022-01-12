package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Vision {

    private static Vision instance;
    public static Vision getInstance() {
        if(instance==null)
            synchronized(Vision.class){
                if(instance==null)
                    instance=new Vision();
            }
        return instance;
    }

    private double FOV =60;
    private int width = 360;
    private double DPP = FOV / width; // degree per pixel
    
    private int x_mid = width / 2;

    private NetworkTableEntry tarX;
    private NetworkTableEntry tarY;

    private boolean updated;

    private double prevTarX;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable nt = inst.getTable("Vision");

    public Vision(){
        tarX = nt.getEntry("target_x");
        tarY = nt.getEntry("target_y");
        
        updated = true;

    }

    public double getTar(){
        tarX = nt.getEntry("target_x");
        double[] tarXs = tarX.getDoubleArray(new double[1]);

        double primaryTarX = 0.0;

        if(tarXs.length!= 0) primaryTarX = tarXs[0];
        
         // a persontage distance from the center

        // double TarAngle = (primaryTarX - x_mid) * DPP ; // angle to turn, clockwise is positive

        double tarAngle = FOV / 2 * primaryTarX;

        System.out.println(tarAngle);

        prevTarX = tarAngle;
        if(prevTarX == tarAngle){
            updated = false;
        }else{
            prevTarX = tarAngle;
            updated = true;
        }

        return tarAngle;
    }

    public boolean hasUpdated(){ 
        return updated;
    }
}