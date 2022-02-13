// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax shooterMaster, shooterSlave;
    private RelativeEncoder masterEncoder, slaveEncoder;
    private SparkMaxPIDController masterPIDController;

    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
     interpolator = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(100);

     
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable nt = inst.getTable("LiveWindow/ShooterSubsystem");
    private NetworkTableEntry RPMEntry = nt.getEntry("RPM");

    public ShooterSubsystem() {

        shooterMaster = new CANSparkMax(20, MotorType.kBrushless);
        shooterSlave = new CANSparkMax(21, MotorType.kBrushless);

        shooterMaster.setInverted(false);
        shooterMaster.setClosedLoopRampRate(0.3);
        shooterMaster.setIdleMode(IdleMode.kCoast);

        shooterSlave.follow(shooterMaster,true);

        masterEncoder = shooterMaster.getEncoder();
        masterEncoder.setVelocityConversionFactor(1);

        slaveEncoder = shooterSlave.getEncoder();
        slaveEncoder.setVelocityConversionFactor(1);

        masterPIDController = shooterMaster.getPIDController();
        masterPIDController.setP(Constants.SHOOTER_GAINS.kP);
        masterPIDController.setI(Constants.SHOOTER_GAINS.kI);
        masterPIDController.setD(Constants.SHOOTER_GAINS.kD);

        masterPIDController.setFF(Constants.SHOOTER_GAINS.kFF);

        masterPIDController.setFeedbackDevice(masterEncoder);

        setRPM(0);

        for(double[] t : Constants.shooterData){
            interpolator.put(new InterpolatingDouble(t[0]), new InterpolatingDouble(t[1]));
        }
    }

    public void setRPM(){
        setRPM(6000);
    }
    public void setRPM(double RPM){

        double feedForward = Constants.Shooter_ks * Math.signum(RPM) + RPM * Constants.Shooter_kv;

        masterPIDController.setReference(RPM, ControlType.kVelocity, 
        0, feedForward);

        // shooterMaster.set(1);
        // shooterSlave.set(1);
    }

    public double getRPM(){
        return masterEncoder.getVelocity();
    }

    public double distanceToRPM(double distance){
        return interpolator.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    public void disabled(){
        setRPM(0);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        System.out.println("Shooter Master RPM: " + masterEncoder.getVelocity());
        System.out.println("Shooter Slave RPM: " + slaveEncoder.getVelocity());

        RPMEntry.setDouble(getRPM());
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
