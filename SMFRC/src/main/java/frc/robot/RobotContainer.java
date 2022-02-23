// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AimLock;
import frc.robot.commands.Anchor;
import frc.robot.commands.AutoInit;
import frc.robot.commands.DisabledCommand;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.Eject;
import frc.robot.commands.FindGoal;
import frc.robot.commands.GoToBall;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.composite.GetAutoIntake;
import frc.robot.commands.composite.GetAutoShoot;
import frc.robot.commands.getAutoTrajectory;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeCamera;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterCamera;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Joystick js1 = new Joystick(0);
  public Joystick js2 = new Joystick(1);

  private DriveSubsystem drive = new DriveSubsystem();
  private ShooterCamera shooterCamera = new ShooterCamera();
  private IntakeCamera intakeCamera = new IntakeCamera();
  private IntakeSubsystem intake = new IntakeSubsystem();
  private IndexerSubsystem indexer = new IndexerSubsystem();
  private ShooterSubsystem shooter = new ShooterSubsystem();
  private ClimberSubsystem climber = new ClimberSubsystem();


  private DriveWithJoystick driveWithJoystick = new DriveWithJoystick(js1, drive);

  private IntakeCommand intakeCommand = new IntakeCommand(intake, indexer);

  private AimLock aimLock = new AimLock(drive, shooterCamera);

  private ShootCommand shoot = new ShootCommand(5, indexer, shooter);

  private Eject eject = new Eject(intake, indexer);

  private FindGoal findGoal = new FindGoal(drive, shooterCamera);

  private Anchor anchor = new Anchor(drive);

  private GoToBall goToBall = new GoToBall(drive, intakeCamera);

  private DisabledCommand disabledCommand = new DisabledCommand(drive, intake, indexer, shooter);

  private AutoInit autoInit = new AutoInit(climber, drive, intake, shooter);

  private RamseteCommand trajectory = getAutoTrajectory.getTrajectory(drive);

  private GetAutoIntake getAutoIntake = new GetAutoIntake(drive, intake, indexer, goToBall);

  private Command autoIntake = getAutoIntake.getCommand();

  private GetAutoShoot getAutoShoot = new GetAutoShoot(findGoal, aimLock, anchor, indexer, shooter);

  private Command autoShoot = getAutoShoot.getCommand();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drive.setDefaultCommand(driveWithJoystick);
    indexer.setDefaultCommand(new RunCommand(() -> indexer.setIndexer(0), indexer));
    climber.setDefaultCommand(new RunCommand(() -> climber.setPower(0), climber));
  }

  // TODO button place to number mapping

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Joystick 1
    //audo shoot
    JoystickButton j1_b1 = new JoystickButton(js1, 1);
    // j1_b1.whenPressed(autoShoot, true);

    // JoystickButton j1_b2 = new JoystickButton(js1, 2); //quick turn in driveWithJoystick

    // intake (press)
    JoystickButton j1_b3 = new JoystickButton(js1, 3);
    j1_b3.whenPressed(intakeCommand);

    //manual shoot (press)
    JoystickButton j1_b4 = new JoystickButton(js1, 4);
    j1_b4.whenPressed(shoot, true);

    //cancel all (press)
    JoystickButton j1_b5 = new JoystickButton(js1, 5);
    j1_b5.whenPressed(disabledCommand, true);

    //intake extend/contract (toggle)
    JoystickButton j1_b10 = new JoystickButton(js1, 10);
    j1_b10.toggleWhenPressed(new StartEndCommand(() -> intake.extendIntake(), () -> intake.contractIntake(), intake), true);

    //intake in/stop (toggle)
    JoystickButton j1_b6 = new JoystickButton(js1, 6);
    j1_b6.toggleWhenPressed(new StartEndCommand(() -> intake.setIntake(0.7), () -> intake.setIntake(0), intake), true);
    
    //eject (hold)
    JoystickButton j1_b9 = new JoystickButton(js1, 9);
    j1_b9.whenHeld(eject, true);

    //indexer up/down (hold)
    JoystickButton j1_b7 = new JoystickButton(js1, 7);
    j1_b7.whenHeld(new StartEndCommand(() -> indexer.setIndexer(0.7), () -> indexer.setIndexer(0), indexer), true);
    JoystickButton j1_b8 = new JoystickButton(js1, 8);
    j1_b8.whenHeld(new StartEndCommand(() -> indexer.setIndexer(-0.7), () -> indexer.setIndexer(0), indexer), true);

    //climer up/down (hold)
    JoystickButton j1_b13 = new JoystickButton(js1, 13);
    j1_b13.whenHeld(new StartEndCommand(() -> climber.setPower(0.7), () -> climber.setPower(0), climber), true);
    JoystickButton j1_b14 = new JoystickButton(js1, 14);
    j1_b14.whenHeld(new StartEndCommand(() -> climber.setPower(-0.7), () -> climber.setPower(0), climber), true);

    JoystickButton j1_b12 = new JoystickButton(js1, 12);
    JoystickButton j1_b15 = new JoystickButton(js1, 15);

    JoystickButton j1_b11 = new JoystickButton(js1, 11);
    JoystickButton j1_b16 = new JoystickButton(js1, 16);

    //Joystick 2

    POVButton j1_p0 = new POVButton(js1, 0);
    j1_p0.whenHeld(new RunCommand(() -> climber.setPower(-1), indexer), false);
    POVButton j1_p315 = new POVButton(js1, 315);
    j1_p315.whenHeld(new RunCommand(() -> climber.setPower(-1), indexer), false);
    POVButton j1_p45 = new POVButton(js1, 45);
    j1_p45.whenHeld(new RunCommand(() -> climber.setPower(-1), indexer), false);

    POVButton j1_p225 = new POVButton(js1, 225);
    j1_p225.whenHeld(new RunCommand(() -> climber.setPower(-1), indexer), false);
    POVButton j1_p135 = new POVButton(js1, 135);
    j1_p135.whenHeld(new RunCommand(() -> climber.setPower(-1), indexer), false);
    POVButton j1_p180 = new POVButton(js1, 180);
    j1_p180.whenHeld(new RunCommand(() -> climber.setPower(-1), indexer), false);


  }

    /* button mapping:
    thrustmaster:
      stick:
        trigger:1
        front:
          3   4
            2
      base:
        left:
          5 6 7
          10 9 8
        right:
          13 12 11
          14 15 16
    
    logitech:
      stick:
        trigger: 1
        left: 2
        front:
          5     6
            3 4
      base:
        7 8
        9 10
        11 12

    POV buttons:
          0
      315   45
    270       90
      225   135
         180
  */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoShoot;
    // return autoIntake;
    return aimLock;
    // return trajectory;
    // return null;
  }

  public Command getDisabledCommand(){
    return disabledCommand;
  }

    public Command getTestCommand(){
      return  null;
      // return new InstantCommand(shooter::setRPM, shooter);
      // return aimLock;
    }
}
