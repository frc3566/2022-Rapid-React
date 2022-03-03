// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterCamera;
import frc.robot.subsystems.ShooterCamera.LastSeen;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AimLock extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem drive;
  private ShooterCamera camera;

  private PIDController pidController = new PIDController(Constants.TURNING_GAINS.kP,
   Constants.TURNING_GAINS.kI, Constants.TURNING_GAINS.kD);

  private double prevUpdateTime = 0;
  private double setpoint;
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem DriveSubsystem
   * @param shooterCamera GoalCamera
   */
  public AimLock(DriveSubsystem driveSubsystem, ShooterCamera shooterCamera) {
    drive = driveSubsystem;
    camera = shooterCamera;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    pidController.setTolerance(0.5);

    setpoint= camera.getTarAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(camera.goalDetected()){
      if(prevUpdateTime != camera.getLastUpdateTime()){
        setpoint = drive.getHeading() + camera.getTarAngle();
        // System.out.println("update turning");
      }
  
      double tarAngle = setpoint - drive.getHeading();
  
      double leftFF = Constants.Drive_ks * Math.signum(tarAngle);
      double rightFF = Constants.Drive_ks * Math.signum(tarAngle); 
  
      double pid = pidController.calculate(drive.getHeading(), setpoint);
  
      MathUtil.clamp(pid, 0, Constants.kMaxSpeed_Drive);
  
      drive.setVelocity((leftFF + pid), -(rightFF + pid));

      System.out.println(pid);
    }else{
      if(camera.lastSeen == LastSeen.RIGHT){
        drive.setVelocity(0.7, -0.7);
      }else{
        drive.setVelocity(-0.7, 0.7);
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(camera.goalDetected() && Math.abs(camera.getTarAngle()) <= 0.3){
      return true;
    }
    return false;
  }
}
