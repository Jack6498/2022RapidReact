// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;
import static frc.surpriselib.UnitConverter.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.surpriselib.wrappers.AHRSWrap;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DriveBase extends SubsystemBase implements Loggable {
  // Gyro
  private AHRSWrap navX = new AHRSWrap(Port.kMXP);

  // Falcons
  private WPI_TalonFX leftLeader = new WPI_TalonFX(leftLeaderCANId);
  private WPI_TalonFX leftFollower = new WPI_TalonFX(leftFollowerCANId);
  private WPI_TalonFX rightLeader = new WPI_TalonFX(rightLeaderCANId);
  private WPI_TalonFX rightFollower = new WPI_TalonFX(rightFollowerCANId);
  private MotorControllerGroup leftMotors = new MotorControllerGroup(leftLeader, leftFollower);
  private MotorControllerGroup rightMotors = new MotorControllerGroup(rightLeader, rightFollower);
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);
  private DifferentialDriveOdometry odometry;

  // Simulators
  private TalonFXSimCollection leftSim = leftLeader.getSimCollection();
  private TalonFXSimCollection rightSim = rightLeader.getSimCollection();
  private DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2), 
    gearRatio, 
    robotMOI, 
    Units.lbsToKilograms(robotMassPounds), 
    Units.inchesToMeters(wheelDiameterInches/2), 
    Units.inchesToMeters(trackWidthInches), 
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
  );
  
  Field2d field2d = new Field2d();
  
  public DriveBase() {
    resetSensors();
    odometry = new DifferentialDriveOdometry(navX.getRotation2d());
    SmartDashboard.putData("Field", field2d);
    diffDrive.setSafetyEnabled(false);
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);
  }

  @Override
  public void periodic() {
    odometry.update(
      navX.getRotation2d(), 
      sensorUnitsToMeters(leftLeader.getSelectedSensorPosition()), 
      sensorUnitsToMeters(rightLeader.getSelectedSensorPosition())
    );
    field2d.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(leftLeader.getMotorOutputVoltage(), rightLeader.getMotorOutputVoltage());
    driveSim.update(0.02);

    leftSim.setIntegratedSensorRawPosition(metersToSensorUnits(driveSim.getLeftPositionMeters()));
    leftSim.setIntegratedSensorVelocity(metersPerSecondToSensorUnits(driveSim.getLeftVelocityMetersPerSecond()));
    rightSim.setIntegratedSensorRawPosition(metersToSensorUnits(driveSim.getRightPositionMeters()));
    rightSim.setIntegratedSensorVelocity(metersPerSecondToSensorUnits(driveSim.getRightVelocityMetersPerSecond()));

    leftSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightSim.setBusVoltage(RobotController.getBatteryVoltage());
  }

  public void drive(double throttle, double turn, boolean squareInputs) {
      diffDrive.arcadeDrive(throttle, turn, squareInputs);
  }

  public void stop() {
    leftMotors.set(0);
    rightMotors.set(0);
  }

  public void setBrakeMode(NeutralMode brakeMode) {
    leftLeader.setNeutralMode(brakeMode);
    leftFollower.setNeutralMode(brakeMode);
    rightLeader.setNeutralMode(brakeMode);
    rightFollower.setNeutralMode(brakeMode);
  }

  private void resetSensors() {
    navX.reset();
    leftLeader.setSelectedSensorPosition(0);
    leftFollower.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    rightFollower.setSelectedSensorPosition(0);
  }

}
