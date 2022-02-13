// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AimLock;
import frc.robot.commands.DisabledCommand;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.RunTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterCamera;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TimerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick JS = new Joystick(0);

  private TimerSubsystem timer = new TimerSubsystem();
  private DriveSubsystem drive = new DriveSubsystem();
  private ShooterCamera shooterCamera = new ShooterCamera();
  // private ShooterSubsystem shooter = new ShooterSubsystem();

  private DriveWithJoystick driveWithJoystick = new DriveWithJoystick(JS, drive);
  private RunTrajectory runTrajectory = new RunTrajectory(null, drive); //TODO test trajectory following

  private AimLock aimLock = new AimLock(drive, shooterCamera);

  // private DisabledCommand disabledCommand = new DisabledCommand(drive, shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drive.setDefaultCommand(driveWithJoystick);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return null;
    // return new InstantCommand(shooter::setRPM, shooter);
    return aimLock;
  }

  // public Command getDisabledCommand(){
    // return disabledCommand;
  // }
}
