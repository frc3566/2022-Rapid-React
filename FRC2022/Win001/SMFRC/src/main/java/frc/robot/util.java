package frc.robot;

public class Util {
    public static double deadband(double val, double lower) {

        return (Math.abs(val) > Math.abs(lower) ? val : 0.0);
    }

    public static double pid(Double error, Double integral, Double prevError, Double dt, Double P, Double I, Double D){
        //integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        double derivative = (error - prevError) / dt;
        double outPut = P*error + I*integral + D*derivative;
        return outPut;
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
