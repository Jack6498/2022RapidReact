// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.surpriselib;

import frc.robot.Constants;

/** Add your docs here. */
public class DriveSignal {
    public double leftMotor;
    public double rightMotor;
    public boolean brakeMode;

    public DriveSignal(double left, double right, boolean brakeOn)
    {
        this.leftMotor = left;
        this.rightMotor = right;
        this.brakeMode = brakeOn;
    }

    public DriveSignal(double left, double right)
    {
        this(left, right, true);
    }

    public static DriveSignal NEUTRAL = 
    new DriveSignal(0, 0, false);
    public static DriveSignal BRAKE = 
    new DriveSignal(0, 0, true);

    @Override
    public String toString()
    {
        return String.format("L: %.2f, R: %.2f, B: %s", leftMotor, rightMotor, brakeMode);
    }

}
