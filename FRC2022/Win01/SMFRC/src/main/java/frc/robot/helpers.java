package frc.robot;

public class helpers {
    public double mapValue(double inLow, double inHigh, double outLow, double outHigh, double ip){
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
