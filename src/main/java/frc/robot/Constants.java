// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class DriveConstants {
    public static final int DRIVE_FRONT_LEFT = 4;
    public static final int DRIVE_FRONT_RIGHT = 6;
    public static final int DRIVE_BACK_LEFT = 12;
    public static final int DRIVE_BACK_RIGHT = 5;
    public static final int LEFT_ENCODER_A = 1;
    public static final int LEFT_ENCODER_B = 2;
    public static final int DRIVE_AXIS = 1;
    public static final int TURN_AXIS = 0;
    public static final double TURN_PROPORTION = 0.7;
    public static final double DRIVE_SLOW = 0.5;
    public static final double TURN_SLOW = 0.8;
    public static final int SLOW_BUTTON = 1;
    public static final int NORMAL_BUTTON = 2;

    public static final int ENCODER_CPR = 1024;
    public static final double WHEEL_DIAMETER = 0.15;
    public static final double ENCODER_DISTANCE_PER_PULSE = 
        (WHEEL_DIAMETER * Math.PI) / (double) ENCODER_CPR;

    public static final String GYRO_SENSOR_NAME = "navX-Sensor[0]";
    public static final double TRACK_WIDTH_METRES = 0.69;
    public static final double WHEEL_DIAMETER_METRES = 0.15;

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
    public static final double kDriveGearing = 8;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // These two values are "angular" kV and kA
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
      LinearSystemId.identifyDrivetrainSystem(
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter,
        kvVoltSecondsPerRadian,
        kaVoltSecondsSquaredPerRadian);

  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_LEFT = 1;
    public static final int ELEVATOR_RIGHT = 2;
    public static final int LIMIT_SWITCH_PORT = 0;
    public static final int ELEVATOR_UP_BUTTON = 4;
    public static final int ELEVATOR_DOWN_BUTTON = 2;
    public static final double ELEVATOR_UP_SPEED = -0.6;
    public static final double ELEVATOR_DOWN_SPEED = 0.3;
  }

  public static class IntakeConstants {
    public static final int INTAKE = 3;
    public static final int CUBE_IN_BUTTON = 6;
    public static final int CONE_IN_BUTTON = 8;
    public static final int INTAKE_OFF_BUTTON = 3;
    public static final double CUBE_IN_SPEED = 0.4;
    public static final double CUBE_IN_STALL = 0.2;
    public static final double CONE_IN_SPEED = -0.5;
    public static final double CONE_IN_STALL = -0.2;
  }

  public static class RotationConstants {
    public static final int ROTATION = 13;
    public static final double ROTATION_STALL = 0.15;
    public static final int ROTATION_UP_BUTTON = 5;
    public static final int ROTATION_DOWN_BUTTON = 7;
    public static final double ROTATION_UP_SPEED = 0.95;
    public static final double ROTATION_DOWN_SPEED = -0.55;
  }

  public static class AutoConstants {
    //Rotation
    public static final double ROTATION_OUT_TIME = 0.3;
    public static final double ROTATION_UP_TIME = 0.7;
    public static final double ROTATION_DOWN_TIME = 0.7;
    public static final double ROTATION_OUT_SPEED = -0.5;
    public static final double ROTATION_UP_SPEED = 0.9;
    public static final double ROTATION_DOWN_SPEED = -0.6;
  
    //Intake
    public static final double INTAKE_SHOOT_TIME = 0.5;
    public static final double INTAKE_SHOOT_SPEED = -0.9;

    //Elevator
    public static final double ELEVATOR_ALIGN_TIME = 0.1;
    public static final double ELEVATOR_DOWN_TIME = 1.5;
    public static final double ELEVATOR_UP_SPEED = -0.5;
    public static final double ELEVATOR_ALIGN_SPEED = 0.15;
    public static final double ELEVATOR_DOWN_SPEED = 0.4;

    //Drive
    public static final double ENGAGE_DRIVE_TIME = 2.0;
    public static final double AUTO_DRIVE_SPEED = 0.65;
    public static final double ENGAGE_DRIVE_SPEED = 0.55;
    public static final double ENGAGE_ENCODER_VALUE = -1240;
    public static final double MOBILITY_ENCODER_VALUE = -2480;
    public static final double MOBILITY_DRIVE_TIME = 3.5;
  }
}
