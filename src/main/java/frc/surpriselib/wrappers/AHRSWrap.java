// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.surpriselib.wrappers;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Robot;

/** Add your docs here. */
public class AHRSWrap implements Gyro, Sendable {
    AHRS realGyro;
    SimDouble simAngle;
    SimDouble simRate;
    int deviceHandle;
    
    public AHRSWrap(SPI.Port port) {
        if (Robot.isReal()) {
            realGyro = new AHRS(port);
        } else {
            deviceHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            simAngle = new SimDouble(
                SimDeviceDataJNI.getSimValueHandle(deviceHandle, "Yaw")
            );
            simRate = new SimDouble(
                SimDeviceDataJNI.getSimValueHandle(deviceHandle, "Rate")
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
        if (Robot.isReal()) {
            realGyro.reset();
        } else {
            simAngle.set(0);
        }
    }

    public double getRate() {
        if (Robot.isReal()) {
            return realGyro.getRate();
        } else {
            return simRate.get();
        }
    }

    public void setAngle(double d) {
        simAngle.set(d);
    }

    public void calibrate() {
        // AHRS does nothing, so shall we
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Yaw", this::getAngle, null);
    }

    public void close() {
        if (Robot.isReal()) {
            realGyro.close();
        } else {
            SendableRegistry.remove(this);
        }
    }
}
