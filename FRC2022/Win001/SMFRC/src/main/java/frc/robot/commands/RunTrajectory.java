// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RunTrajectory extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DriveSubsystem drive;

  private Trajectory trajectory;

  private double startTime;

// Create a voltage constraint to ensure we don't accelerate too fast
DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.ks_Drive,
            Constants.kv_Drive,
            Constants.ka_Drive),
        Constants.kDriveKinematics,
        10);

// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.kMaxSpeed_Drive,
            Constants.kMaxAcceleration_Drive)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

RamseteController controller = new RamseteController();

  /**
   * Creates a new ExampleCommand.
   * 
   * @param trajectory The trajectory to follow
   * @param driveSubsystem DriveSubsystem
   * 
   */

  public RunTrajectory(Trajectory trajectory, DriveSubsystem driveSubsystem) {

    drive = driveSubsystem;
    this.trajectory = trajectory;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = Timer.getFPGATimestamp() - startTime;

    Trajectory.State goal = trajectory.sample(currTime);
    ChassisSpeeds adjustedSpeeds = controller.calculate(drive.getPose(), goal);

    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.kDriveKinematics.toWheelSpeeds(adjustedSpeeds);
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;

    drive.setVelocity(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
