// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

/** An example command that uses an example subsystem. */
public class ClimbWithJoystick extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private ClimberSubsystem climber;

  private Joystick JS;
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsystem DriveSubsystem
   */
  public ClimbWithJoystick(Joystick joystick, ClimberSubsystem climberSubsystem) {
    climber = climberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);

    JS = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = JS.getRawAxis(0);

    climber.setPower(power);
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
