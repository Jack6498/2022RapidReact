// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.surpriselib;

import edu.wpi.first.math.util.Units;
import static java.lang.Math.*;

public class UnitConverter {
    static double gearRatio;
    static double wheelDiameter;
    static double wheelCircumference;

    public static void setupConstants(double gearing, double wheelSizeInches) {
        gearRatio = gearing;
        wheelDiameter = Units.inchesToMeters(wheelSizeInches);
        wheelCircumference = PI * wheelDiameter;
    }

    // ctre sensor units to rpm
    public static double sensorUnitsToRPM(double units) {
        return (600 * units * gearRatio) / 2048;
    }

    public static double rpmToSensorUnits(int rpm) {
        double units = (rpm / 600) * (2048 / gearRatio);
        return units;
    }

    public static double sensorUnitsToMeters(double units) {
        double rotations = units / 2048;
        double wheelRotations = rotations / gearRatio;
        double positionMeters = wheelRotations * wheelCircumference;
        return positionMeters;
    }

    public static int metersToSensorUnits(double meters) {
        double wheelRotations = meters / wheelCircumference;
        double motorRotations = wheelRotations * gearRatio;
        int units = (int)(motorRotations * 2048);
        return units;
    }

    public static int metersPerSecondToSensorUnits(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond / wheelCircumference;
        double motorRotationsPerSecond = wheelRotationsPerSecond * gearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / 10;
        int sensorCountPer100ms = (int)(motorRotationsPer100ms * 2048);
        return sensorCountPer100ms;
    }

    public static double sensorUnitsToMetersPerSecond(double sensorCountsPerDecisecond) {
        double sensorCountsPerSecond = sensorCountsPerDecisecond * 10;
        double motorRotationsPerSecond = sensorCountsPerSecond / 2048; 
        double wheelRotationsPerSecond = motorRotationsPerSecond / gearRatio; 
        double metersPerSecond = wheelRotationsPerSecond *wheelCircumference;
        return metersPerSecond;
    }

}
