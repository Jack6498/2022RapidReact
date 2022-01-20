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
import frc.surpriselib.wrappers.AHRSSim;
import io.github.oblarg.oblog.Loggable;

public class DriveBase extends SubsystemBase implements Loggable {
  // MOTORS AND DRIVE
  WPI_TalonFX leftLeader, leftFollower, rightLeader, rightFollower;
  MotorControllerGroup leftMotors, rightMotors;
  DifferentialDriveKinematics kinematics;
  DifferentialDriveOdometry odometry;
  DifferentialDrive diffDrive;
  ChassisSpeeds chassisSpeeds;
  WPI_TalonFX[] motors = new WPI_TalonFX[] { leftLeader, leftFollower, rightLeader, rightFollower };

 // SIM OBJECTS
  TalonFXSimCollection leftSim;
  TalonFXSimCollection rightSim;
  AHRSSim gyro;
  DifferentialDrivetrainSim driveSim;
  //@Log
  Field2d field = new Field2d();
  // sim end

  Pose2d robotPose;
  
  double ramp = 2;

  /** Creates a new DriveBase. */
  public DriveBase() {
    // motors
    leftLeader = new WPI_TalonFX(leftLeaderCANId);
    leftFollower = new WPI_TalonFX(leftFollowerCANId);
    rightLeader = new WPI_TalonFX(rightLeaderCANId);
    rightFollower = new WPI_TalonFX(rightFollowerCANId);
    leftMotors = new MotorControllerGroup(leftLeader, leftFollower);
    rightMotors = new MotorControllerGroup(rightLeader, rightFollower);
    leftLeader.configOpenloopRamp(ramp);
    leftFollower.configOpenloopRamp(ramp);
    rightLeader.configOpenloopRamp(ramp);
    rightFollower.configOpenloopRamp(ramp);
    // gyro
    gyro = new AHRSSim(Port.kMXP);

    // wpi
    resetEncoders();
    gyro.reset();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(trackWidthInches));
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    robotPose = new Pose2d();
    if (Robot.isReal()) {
      diffDrive = new DifferentialDrive(leftMotors, rightMotors);
    }
    // sim
    driveSim = new DifferentialDrivetrainSim(
      DCMotor.getFalcon500(2),
      gearRatio,
      3.2, // moment of inertia from cad model
      26.5, // mass of robot in kg
      Units.inchesToMeters(wheelDiameterInches), // wheel diameter in meters
      1.0, // robot track width in meters
      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );
    leftSim = new TalonFXSimCollection(leftLeader);
    rightSim = new TalonFXSimCollection(rightLeader);
    SmartDashboard.putData("Field", field);
  }

  public void setStartingPose(Pose2d pose)
  {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  private void resetEncoders()
  {
    leftLeader.setSelectedSensorPosition(0);
    leftFollower.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    rightFollower.setSelectedSensorPosition(0);
  }

  public void setArcadeDrive(double throttle, double turn)
  {
    diffDrive.arcadeDrive(throttle, turn);
    
  }

  public void stop(NeutralMode brake) {
    leftMotors.set(0);
    rightMotors.set(0);
    leftLeader.setNeutralMode(brake);
    leftFollower.setNeutralMode(brake);
    rightLeader.setNeutralMode(brake);
    rightFollower.setNeutralMode(brake);
  }

  @Override
  public void periodic() {
    odometry.update(
      gyro.getRotation2d(), 
      sensorUnitsToMeters(
        leftLeader.getSelectedSensorPosition()
      ), 
      sensorUnitsToMeters(
        rightLeader.getSelectedSensorPosition()
      )
    );
    DifferentialDriveWheelSpeeds wheelSpeeds = 
      new DifferentialDriveWheelSpeeds(
        sensorUnitsToMetersPerSecond(
          leftLeader.getSelectedSensorVelocity()
        ),
        sensorUnitsToMetersPerSecond(
          rightLeader.getSelectedSensorVelocity()
        )
      );
      chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // set inputs
    // USE VOLTAGE
    driveSim.setInputs(
      leftLeader.getMotorOutputVoltage(), 
      rightLeader.getMotorOutputVoltage()
    );
    leftLeader.feed();
    rightLeader.feed();
    // advance by 20ms (Robot loop)
    driveSim.update(0.02);

    // update sensors
    leftSim.setIntegratedSensorRawPosition(
      metersToSensorUnits(
        driveSim.getLeftPositionMeters()
      )
    );
    leftSim.setIntegratedSensorVelocity(
      metersPerSecondToSensorUnits(
        driveSim.getLeftVelocityMetersPerSecond()
      )
    );
    rightSim.setIntegratedSensorRawPosition(
      metersToSensorUnits(
        driveSim.getRightPositionMeters()
      )
    );
    rightSim.setIntegratedSensorVelocity(
      metersPerSecondToSensorUnits(
        driveSim.getRightVelocityMetersPerSecond()
      )
    );
    gyro.setAngle(-driveSim.getHeading().getDegrees());
    
    leftSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightSim.setBusVoltage(RobotController.getBatteryVoltage());
  }
}
