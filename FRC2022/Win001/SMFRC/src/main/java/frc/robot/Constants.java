// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final double L_PulsePerMeter = 21.4032;
    public static final double R_PulsePerMeter = 22.61056;
    
    public static final double FOV =60;
    public static final double pixelWidth = 360;

    public static final double GEAR_RATIO = 11.0 / 50.0 * 24.0 / 50.0; // MotorCnt / WheelCnt;
    public static final double WHEEL_DIAMETER =  6.28 * 0.0254 * Math.PI;
    public static final double ENCODER_UNITpMETER = (1 / GEAR_RATIO) / WHEEL_DIAMETER;
    public static final double RPMpMPS = ENCODER_UNITpMETER * 60.0;

    // drivetrain physical characteristic
    public static final double ks = 0.101;
    public static final double kv = 2.44;
    public static final double ka = 0.38;

    // shooter subsystem
    public static final double SHOOTER_KS = 0; //TODO tuning

    //motorcontroller gains
    public static final Gains DRIVETRAIN_VELOCITY_GAINS = new Gains(
        1 / 2 * RPMpMPS, 1 / 100 * RPMpMPS, 0, 0, 0.2 * Constants.RPMpMPS, 1);

    public static class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kF;
        public final double kIzone;
        public final double kPeakOutput;
    
        public Gains(double _kP, double _kI, double _kD, double _kF, double _kIzone, double _kPeakOutput) {
          kP = _kP;
          kI = _kI;
          kD = _kD;
          kF = _kF;
          kIzone = _kIzone;
          kPeakOutput = _kPeakOutput;
        }
      }
}
