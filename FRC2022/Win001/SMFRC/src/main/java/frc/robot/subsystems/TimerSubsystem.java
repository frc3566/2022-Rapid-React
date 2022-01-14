// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class TimerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private Timer timer;

  private double dt;

  private double timeSinceT;

  private double T;

  private double prevT;

  private double currT;

  public TimerSubsystem() {
    timer = new Timer();
  }

  public void start(){
    timer.reset();
    T = Timer.getFPGATimestamp();
    return;
  }

  public double getDT(){
    return dt;
  }

  public double getStartTime(){
    return T;
  }

  public double getTimeSinceT(){
    return timeSinceT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    prevT = currT;
    currT = Timer.getFPGATimestamp();
    dt = currT - prevT;
    timeSinceT = currT - T;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
