// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveOpenLoop extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveBase drive;
  DoubleSupplier forward, turn, reverse;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveOpenLoop(DriveBase subsystem, DoubleSupplier throttle, DoubleSupplier turn, DoubleSupplier reverse) {
    drive = subsystem;
    this.forward = throttle;
    this.turn = turn;
    this.reverse = reverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // going forwards
    if (!getReverse()) {
      drive.drive(forward.getAsDouble(), turn.getAsDouble(), true);
    } else {
      drive.drive(reverse.getAsDouble(), turn.getAsDouble(), true);
    }
  }

  private boolean getReverse() {
    return reverse.getAsDouble() > forward.getAsDouble();
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
