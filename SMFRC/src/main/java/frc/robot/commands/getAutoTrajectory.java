package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class getAutoTrajectory{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

// Create a voltage constraint to ensure we don't accelerate too fast
private static DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.Drive_ks,
            Constants.Drive_kv,
            Constants.Drive_ka),
        Constants.kDriveKinematics,
        10);

// Create config for trajectory
private static TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.kMaxSpeed_Drive,
            Constants.kMaxAcceleration_Drive)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

// private static Trajectory trajectory = 
// TrajectoryGenerator.generateTrajectory(
//     // Start at the origin facing the +X direction
//     new Pose2d(0, 0, new Rotation2d(0)),
//     // Pass through these two interior waypoints, making an 's' curve path
//     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//     // End 3 meters straight ahead of where we started, facing forward
//     new Pose2d(3, 0, new Rotation2d(0)),
//     // Pass config
//     config);

private static RamseteController controller;

private static String trajectoryJSON = "1.wpilib.json";
private static Trajectory trajectory = new Trajectory();

  public static RamseteCommand getTrajectory(DriveSubsystem driveSubsystem) {
    
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    return new RamseteCommand(
      trajectory,
      driveSubsystem::getPose,
      new RamseteController(2.0, 0.7),
      new SimpleMotorFeedforward(Constants.Drive_ks,
                                Constants.Drive_kv,
                                Constants.Drive_ka),
      Constants.kDriveKinematics,
      driveSubsystem::getWheelSpeeds,
      new PIDController(Constants.DRIVETRAIN_VELOCITY_GAINS.kP, 0, 0),
      new PIDController(Constants.DRIVETRAIN_VELOCITY_GAINS.kP, 0, 0),
      // RamseteCommand passes volts to the callback
      driveSubsystem::setVoltage,
      driveSubsystem
    );
  }
}
