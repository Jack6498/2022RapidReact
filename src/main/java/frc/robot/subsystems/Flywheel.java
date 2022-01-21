// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

public class Flywheel extends SubsystemBase {
  // neo
  CANSparkMax neo = new CANSparkMax(flywheelCANId, MotorType.kBrushless);
  SparkMaxPIDController pid = neo.getPIDController();
  int maxRPM = 5780;
  /** Creates a new example. */
  public Flywheel() {
    neo.setClosedLoopRampRate(1);
    neo.setIdleMode(IdleMode.kCoast);
  }

  public void runLowGoal() {
    setOutput(0.5);
  }

  public void runHighGoal() {
    setOutput(0.75);
  }

  public void setOutput(double percent) {
    pid.setReference(percentToRPM(percent), ControlType.kVelocity);
  }

  private double percentToRPM(double percent) {
    return percent * maxRPM;
  }
}
