// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static enum ballColors {RED, BLUE}

    public static ballColors ballColor = ballColors.RED;
    
    public static final double L_PulsePerMeter = 21.4032;
    public static final double R_PulsePerMeter = 22.61056;

//TODO meaure these
    public static final double GEAR_RATIO = 11.0 / 50.0 * 24.0 / 50.0; // MotorCnt / WheelCnt;
    public static final double WHEEL_DIAMETER =  6.28 * 0.0254 * Math.PI;
    public static final double ENCODER_UNITpMETER = (1 / GEAR_RATIO) / WHEEL_DIAMETER;
    public static final double RPMpMPS = ENCODER_UNITpMETER * 60.0;

    // drivetrain physical characteristic
    public static final double kTrackwidthMeters = 0.7112 / 0.93;
    public static final DifferentialDriveKinematics kDriveKinematics = 
    new DifferentialDriveKinematics(kTrackwidthMeters);
//TODO sysid
    public static final double Drive_ks = 0.101;
    public static final double Drive_kv = 2.44;
    public static final double Drive_ka = 0.38;

    public static final double kMaxSpeed_Drive = 1.5; 
    public static final double kMaxAcceleration_Drive = 1.5;

    //turning control
    public static final Gains TURNING_GAINS = new Gains(0.135, 0, 0.00165, 0, 0, 1);

    // shooter subsystem
    public static final double Shooter_ks = 0.004588166667;
    public static final double Shooter_kv = 0.0021445;
    public static final double Shooter_ka = 0.0001474133333;

    public static final Gains SHOOTER_GAINS = new Gains(0.0004,0,0.0001,0,0,12); // prevP 0.00041032
    // public static final Gains SHOOTER_GAINS = new Gains(1,0,0,0,0,12);


    //ten points in the frommat (meter, RPM) that are used to interpolate RPM for certain distance
    public static final double[][] shooterData = 
    {{0,0},{0,0},{0,0},{0,0},{0,0},
    {0,0},{0,0},{0,0},{0,0},{0,0}};

    //motorcontroller gains
    public static final Gains DRIVETRAIN_VELOCITY_GAINS = new Gains(
        1 / 2 * RPMpMPS, 1 / 100 * RPMpMPS, 0, 0, 0.2 * Constants.RPMpMPS, 1);

    public static class Gains {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kFF;
        public final double kIzone;
        public final double kPeakOutput;
    
        public Gains(double _kP, double _kI, double _kD, double _kFF, double _kIzone, double _kPeakOutput) {
          kP = _kP;
          kI = _kI;
          kD = _kD;
          kFF = _kFF;
          kIzone = _kIzone;
          kPeakOutput = _kPeakOutput;
        }
      }
}
