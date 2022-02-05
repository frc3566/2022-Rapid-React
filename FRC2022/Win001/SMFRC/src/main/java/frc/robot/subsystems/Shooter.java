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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {
    private CANSparkMax shooterMaster, shooterSlave;
    private RelativeEncoder encoder;
    private SparkMaxPIDController shooterPIDController;

    private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
     interpolator = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(100);

    /** Creates a new ExampleSubsystem. */
    public Shooter() {

        shooterMaster = new CANSparkMax(20, MotorType.kBrushless);
        shooterSlave = new CANSparkMax(21, MotorType.kBrushless);

        shooterMaster.setInverted(false);
        shooterMaster.setClosedLoopRampRate(0.3);
        shooterMaster.setIdleMode(IdleMode.kCoast);

        shooterSlave.setInverted(false);
        shooterSlave.setClosedLoopRampRate(0.3);
        shooterSlave.setIdleMode(IdleMode.kCoast);
        
        encoder = shooterMaster.getEncoder();
        encoder.setVelocityConversionFactor(1);

        shooterPIDController = shooterMaster.getPIDController();
        shooterPIDController.setP(Constants.SHOOTER_GAINS.kP);
        shooterPIDController.setI(Constants.SHOOTER_GAINS.kI);
        shooterPIDController.setD(Constants.SHOOTER_GAINS.kD);
        shooterPIDController.setFF(Constants.SHOOTER_GAINS.kFF);
        shooterPIDController.setFeedbackDevice(encoder);
        // shooterPIDController.setFF(Constants.SHOOTER_GAINS.kFF);
        setRPM(0);

        for(double[] t : Constants.shooterData){
            interpolator.put(new InterpolatingDouble(t[0]), new InterpolatingDouble(t[1]));
        }

        shooterSlave.follow(shooterMaster);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        System.out.println("Shooter RPM: " + encoder.getVelocity());
    }

    public void setRPM(){
        setRPM(1000);
    }

    public void setRPM(double RPM){

        RPM *= 2;

        double feedForward = Constants.Shooter_ks;

        shooterPIDController.setReference(RPM, ControlType.kVelocity, 
        0, feedForward);
    }

    public double getRPM(){
        return encoder.getVelocity();
    }

    public double distanceToRPM(double distance){
        return interpolator.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
