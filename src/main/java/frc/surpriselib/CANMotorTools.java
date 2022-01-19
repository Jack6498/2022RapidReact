// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.surpriselib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class CANMotorTools {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    public void setupMotor(TalonFX falcon, double openLoopRamp)
    {
        driveConfig.openloopRamp = openLoopRamp;
        falcon.configAllSettings(driveConfig);
    }

    public void setupMotor(TalonFX falcon, TalonFXConfiguration config)
    {
        falcon.configAllSettings(config);
    }

    public static WPI_TalonFX PhoenixToWPI(TalonFX falcon, double openLoopRamp, NeutralMode neutralMode, int statusFramePeriod, FeedbackDevice feedbackSensor, boolean sensorPhase, boolean inverted)
    {
        WPI_TalonFX wpiFalcon = new WPI_TalonFX(falcon.getDeviceID());
        wpiFalcon.configOpenloopRamp(openLoopRamp);
        wpiFalcon.setNeutralMode(neutralMode);
        wpiFalcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, statusFramePeriod);
        wpiFalcon.configSelectedFeedbackSensor(feedbackSensor);
        wpiFalcon.setSensorPhase(sensorPhase);
        wpiFalcon.setInverted(inverted);
        return wpiFalcon;
    }

    public void setupFalconPIDSlot(final TalonFX falcon, final TalonFXConfiguration talon, final int slotId, final double P, final double I, final double D, final double F, final int iZone, final double rampRate)
    {
        SlotConfiguration slot = new SlotConfiguration();
        slot.kP = P;
        slot.kI = I;
        slot.kD = D;
        slot.kF = F;
        slot.integralZone = iZone;
        talon.closedloopRamp = rampRate;
        switch(slotId)
        {
            case 0:
                talon.slot0 = slot;
                break;
            case 1:
                talon.slot1 = slot;
                break;
            case 2:
                talon.slot2 = slot;
                break;
            case 3:
                talon.slot3 = slot;
                break;
            default:
                break;
        }
        falcon.configAllSettings(talon);
    }

}
