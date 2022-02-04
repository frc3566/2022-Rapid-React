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

public class Shooter extends SubsystemBase {
    private CANSparkMax shooterMaster;
    private RelativeEncoder encoder;
    private SparkMaxPIDController shooterPIDController;

    /** Creates a new ExampleSubsystem. */
    public Shooter() {

        shooterMaster = new CANSparkMax(20, MotorType.kBrushless);
        shooterMaster.setInverted(false);
        shooterMaster.setClosedLoopRampRate(0.3);
        shooterMaster.setIdleMode(IdleMode.kCoast);
        
        encoder = shooterMaster.getEncoder();
        shooterPIDController = shooterMaster.getPIDController();
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        //spin(0.90);
    }

    public void setRPM(){
        setRPM(5000);
    }

    public void setRPM(double RPM){
        shooterPIDController.setReference(RPM, ControlType.kVelocity,
        0, Constants.SHOOTER_KS);
        System.out.println("Shooter RPM: " + encoder.getVelocity());
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
