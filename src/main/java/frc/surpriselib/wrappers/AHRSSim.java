// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.surpriselib.wrappers;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;

/** Add your docs here. */
public class AHRSSim {
    AHRS realGyro;
    SimDouble simAngle;
    
    public AHRSSim(SPI.Port port) {
        if (Robot.isReal()) {
            realGyro = new AHRS(port);
        } else {
            simAngle = new SimDouble(
                SimDeviceDataJNI.getSimValueHandle(
                    SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor"), "Yaw"
                )
            );
        }
    }

    public Rotation2d getRotation2d() {
        if (Robot.isReal()) {
            return realGyro.getRotation2d();
        } else {
            return Rotation2d.fromDegrees(getAngle());
        }
    }

    public double getAngle() {
        if (Robot.isReal()) {
            return realGyro.getAngle();
        } else {
            return simAngle.get();
        }
    }

    public void reset() {
        simAngle.set(0);
    }

    public void setAngle(double d) {
        simAngle.set(d);
    }
}
