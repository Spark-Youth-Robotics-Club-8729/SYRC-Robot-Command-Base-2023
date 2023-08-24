// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final Encoder encoderRightDrive = new Encoder(0, 0); // TODO fill in these with right encoder channels

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  // simulation stuff
  private final Field2d field2d;
  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;
  private final SimDeviceSim gyroSim;
  private final DifferentialDrivetrainSim drivetrainSim;

  public DriveSubsystem() {
    driveFrontLeft.setNeutralMode(NeutralMode.Brake);
    driveFrontRight.setNeutralMode(NeutralMode.Brake);
    driveBackLeft.setNeutralMode(NeutralMode.Brake);
    driveBackRight.setNeutralMode(NeutralMode.Brake);
    driveRight.setInverted(true);

    resetEncoders();
    odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(getGyroYaw()),
        encoderLeftDrive.getDistance(),
        encoderRightDrive.getDistance());

    if (RobotBase.isSimulation()) {
      field2d = new Field2d();
      leftEncoderSim = new EncoderSim(encoderLeftDrive);
      rightEncoderSim = new EncoderSim(encoderRightDrive);
      gyroSim = new SimDeviceSim("NavX-Sensor[0]"); // TODO verify device tag?
      drivetrainSim = new DifferentialDrivetrainSim(
          LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3), // TODO replace these with actual values
          DCMotor.getFalcon500(2), // TODO is this actually the right motors??
          8, // TODO fill in actual gearing
          0.76, // TODO fill in actual track width
          0.0508, // TODO fill in actual wheel radius
          null);

      SmartDashboard.putData("Field", field2d);
    } else {
      field2d = null;
      leftEncoderSim = null;
      rightEncoderSim = null;
      gyroSim = null;
      drivetrainSim = null;
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
        Rotation2d.fromDegrees(getGyroYaw()),
        leftEncoderSim.getDistance(),
        rightEncoderSim.getDistance()
    );

    if (field2d != null) {
      field2d.setRobotPose(getPose());
    }
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.setInputs(
        driveLeft.get() * RobotController.getBatteryVoltage(),
        driveRight.get() * RobotController.getBatteryVoltage()
    );
    drivetrainSim.update(0.02);

    leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
    gyroSim.getDouble("Yaw").set(drivetrainSim.getHeading().getDegrees());
  }

  private Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setMotor(double forwardSpeed, double turnSpeed) {
    driveRobot.arcadeDrive(forwardSpeed, turnSpeed);
  }
}
