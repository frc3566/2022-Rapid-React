package frc.robot.util;

public class Util {
    public static double deadband(double val, double lower) {

        return (Math.abs(val) > Math.abs(lower) ? val : 0.0);
    }

    public static double mapValue(double inLow, double inHigh, double outLow, double outHigh, double ip){
        double n = ip;
        if(n>inHigh){
          n = inHigh;
        }else if(n < inLow){
          n = inLow;
        }
    
        n = n - inLow;
        n = n / (inHigh-inLow);
        n = n * (outHigh-outLow);
        n = n + outLow;
    
        return n;
      }
}
