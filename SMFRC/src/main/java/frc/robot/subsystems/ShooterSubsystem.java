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
    private SparkMaxPIDController masterPIDController, slavePIDController;

    private double k = 1050/1000;

    private double fieldCorrection;

    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
     interpolator = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(100);

     
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable nt = inst.getTable("LiveWindow/ShooterSubsystem");
    private NetworkTableEntry RPMEntry = nt.getEntry("RPM");
    private NetworkTableEntry FieldCorrectionEntry = nt.getEntry("field_correction");

    public ShooterSubsystem() {

        shooterMaster = new CANSparkMax(20, MotorType.kBrushless);
        shooterSlave = new CANSparkMax(21, MotorType.kBrushless);

        shooterMaster.setInverted(true);
        shooterMaster.setClosedLoopRampRate(0.3);
        shooterMaster.setIdleMode(IdleMode.kCoast);

        shooterMaster.setInverted(true);
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

        slavePIDController = shooterSlave.getPIDController();
        slavePIDController.setP(Constants.SHOOTER_GAINS.kP);
        slavePIDController.setI(Constants.SHOOTER_GAINS.kI);
        slavePIDController.setD(Constants.SHOOTER_GAINS.kD);

        slavePIDController.setFF(Constants.SHOOTER_GAINS.kFF);

        slavePIDController.setFeedbackDevice(slaveEncoder);

        setRPM(0);

        for(double[] t : Constants.shooterData){
            interpolator.put(new InterpolatingDouble(t[0]), new InterpolatingDouble(t[1]));
        }

        fieldCorrection = 2950;
    }

    public void setRPM(double RPM){

        // RPM += fieldCorrection;

        double feedForward = Constants.Shooter_ks * Math.signum(RPM) + RPM * Constants.Shooter_kv;

        masterPIDController.setReference(RPM, ControlType.kVelocity, 0, feedForward);

        slavePIDController.setReference(RPM * k, ControlType.kVelocity, 0, feedForward);

        // shooterMaster.set(1);
        // shooterSlave.set(1);
    }

    public void setPower(double power){
        shooterMaster.set(power);
        shooterSlave.set(power);
    }

    public double getMasterRPM(){
        return masterEncoder.getVelocity();
    }

    public double getSlaveRPM(){
        return slaveEncoder.getVelocity();
    }

    public double distanceToRPM(double distance){
        return interpolator.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    public double getFieldCorrection(){
        return fieldCorrection;
    }

    public void setFieldCorrection(double correction){
        fieldCorrection = correction;
    }

    public void disabled(){
        setRPM(0);
    }

    public double getManualRPM(){
        return fieldCorrection;
    }

    public void setBrake(boolean isBrake){
        IdleMode sparkMode = isBrake? IdleMode.kBrake : IdleMode.kCoast;
        shooterMaster.setIdleMode(sparkMode);
    };

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    
        System.out.println("Shooter Master RPM: " + masterEncoder.getVelocity());
        System.out.println("Shooter Slave RPM: " + slaveEncoder.getVelocity());

        RPMEntry.setDouble(getMasterRPM());
        FieldCorrectionEntry.setDouble(fieldCorrection);

    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
