// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.surpriselib;

/** Add your docs here. */
public class MathUtils {
    public static double clamp(double value)
    {
        return clamp(value, 1.0, -1.0);
    }

    public static double clamp(double value, double max, double min)
    {
        if (value > max) 
        {
            return max;
        } 
        else if (value < min)
        {
            return min;
        }
        else
        {
            return value;
        }
    }

    public static double applyDeadband(double value, double deadband)
    {
        if (Math.abs(value) > deadband) 
        {
            if (value > 0.0) 
            {
                return (value - deadband) / (1.0 - deadband);
            }
            else
            {
                return (value + deadband) / (1.0 - deadband);
            }
        }
        else
        {
            return 0.0;
        }
    }

    public static double[] normalize(double[] motorSpeeds)
    { 
        double max = Math.abs(motorSpeeds[0]);
        boolean normFlag = max > 1; // do we normalize or just pass it back?
        for (int i = 1; i < motorSpeeds.length; i++) {
            if (Math.abs(motorSpeeds[i]) > max) { // set max to biggest magnitude motor speed
                max = Math.abs(motorSpeeds[i]);
                normFlag = max > 1;
            }
        }
        if (normFlag) { // normalize speeds
            for (int i = 0; i < motorSpeeds.length; i++) {
                motorSpeeds[i] /= max;
            }
        }
        return motorSpeeds;
    }
}
