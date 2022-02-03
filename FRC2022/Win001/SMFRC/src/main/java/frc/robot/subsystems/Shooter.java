// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;

    /** Creates a new ExampleSubsystem. */
    public Shooter() {
        motor = new CANSparkMax(20, MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        spin(1);
    }

    public void spin(int RPM){
        motor.set(RPM);
        System.out.println("RPM: " + encoder.getVelocity());
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
