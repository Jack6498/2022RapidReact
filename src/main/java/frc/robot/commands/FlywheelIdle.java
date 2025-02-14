// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class FlywheelIdle extends CommandBase {
  Flywheel flywheel;
  /** Creates a new FlywheelIdle. */
  public FlywheelIdle(Flywheel wheel) {
    addRequirements(wheel);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setOutput(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
