// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private final WPI_VictorSPX driveFrontLeft = new WPI_VictorSPX(DriveConstants.DRIVE_FRONT_LEFT);
  private final WPI_VictorSPX driveFrontRight = new WPI_VictorSPX(DriveConstants.DRIVE_FRONT_RIGHT);
  private final WPI_VictorSPX driveBackLeft = new WPI_VictorSPX(DriveConstants.DRIVE_BACK_LEFT);
  private final WPI_VictorSPX driveBackRight = new WPI_VictorSPX(DriveConstants.DRIVE_BACK_RIGHT);

  private final MotorControllerGroup driveLeft = new MotorControllerGroup(driveFrontLeft, driveBackLeft);
  private final MotorControllerGroup driveRight = new MotorControllerGroup(driveFrontRight, driveBackRight);
  private final DifferentialDrive driveRobot = new DifferentialDrive(driveLeft, driveRight);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final Encoder encoderLeftDrive = new Encoder(DriveConstants.LEFT_ENCODER_A, DriveConstants.LEFT_ENCODER_B);
  private final Encoder encoderRightDrive = new Encoder(7, 8, true); // TODO fill in these with right encoder channels

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  // simulation stuff
  private final Field2d fieldSim;
  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;
  private final SimDeviceSim gyroSim;
  private final DifferentialDrivetrainSim drivetrainSimulator;

  public DriveSubsystem() {
    driveFrontLeft.setNeutralMode(NeutralMode.Brake);
    driveFrontRight.setNeutralMode(NeutralMode.Brake);
    driveBackLeft.setNeutralMode(NeutralMode.Brake);
    driveBackRight.setNeutralMode(NeutralMode.Brake);
    driveRight.setInverted(true);

    // Sets the distance per pulse for the encoders
    encoderLeftDrive.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
    encoderRightDrive.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);

    resetEncoders();
    odometry =
        new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()),
            encoderLeftDrive.getDistance(),
            encoderRightDrive.getDistance());

    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      drivetrainSimulator =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.TRACK_WIDTH_METRES,
              DriveConstants.WHEEL_DIAMETER_METRES / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

      // The encoder and gyro angle sims let us set simulated sensor readings
      leftEncoderSim = new EncoderSim(encoderLeftDrive);
      rightEncoderSim = new EncoderSim(encoderRightDrive);
      gyroSim = new SimDeviceSim(DriveConstants.GYRO_SENSOR_NAME); // TODO verify device tag?

      // the Field2d class lets us visualize our robot in the simulation GUI.
      fieldSim = new Field2d();
      SmartDashboard.putData("Field", fieldSim);
    } else {
      drivetrainSimulator = null;
      leftEncoderSim = null;
      rightEncoderSim = null;
      gyroSim = null;

      fieldSim = null;
    }
  }

  public double getEncoderDrivePosition() {
    return (encoderLeftDrive.getDistance());
  }

  public double getGyroYaw() {
    return (gyro.getYaw());
  }

  public double getGyroPitch() {
    return (gyro.getPitch());
  }

  public double getGyroRoll() {
    return (gyro.getRoll());
  }

  public void resetEncoders() {
    encoderLeftDrive.reset();
    encoderRightDrive.reset();
  }

  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Roll", getGyroRoll());
    SmartDashboard.putNumber("Pitch", getGyroPitch());
    SmartDashboard.putNumber("Yaw", getGyroYaw());
    SmartDashboard.putNumber("Encoder Left Distance", encoderLeftDrive.getDistance());

    odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        leftEncoderSim.getDistance(),
        rightEncoderSim.getDistance()
    );

    if (fieldSim != null) {
      fieldSim.setRobotPose(getPose());
    }
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSimulator.setInputs(
        driveLeft.get() * RobotController.getBatteryVoltage(),
        driveRight.get() * RobotController.getBatteryVoltage()
    );
    drivetrainSimulator.update(0.020);

    leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());

    setGyroHeading(-drivetrainSimulator.getHeading().getDegrees());
  }

  public void setGyroHeading(double gyroHeading) {
    int dev = SimDeviceDataJNI.getSimDeviceHandle(DriveConstants.GYRO_SENSOR_NAME);
    SimDouble gyroSimAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    gyroSimAngle.set(gyroHeading);
  }

  private Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setMotor(double forwardSpeed, double turnSpeed) {
    driveRobot.arcadeDrive(forwardSpeed, turnSpeed);
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw(), 360) * -1.0;
  }
}
