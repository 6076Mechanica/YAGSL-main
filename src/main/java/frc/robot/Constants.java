// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double JOYSTICK_XY_EXP = 2.25;
    public static final double JOYSTICK_Z_EXP = 2.25;
    public static final double JOYSTICK_XY_DEADBAND = 0.1;
    public static final double JOYSTICK_Z_DEADBAND = 0.1;
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final int kDriverControllerPort = 1;
    public static final int kJoystickPort = 0;
  }


  public static class shooterConstants {
    public static final int kNeoCANPort = 1;
    public static final int kNeoFreeSpeed = 5500;
    public static final int kTalonFreeSpeed = 6300;
    public static final int ktolerance = 500;
    public static final int kAmpshot = 10;
    public static final int kSpeakerShot = 25;

    public static final double kS = 0.0005;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double kP = 0.000175;
    public static final double kI = 0.0003;
    public static final double kD = 0;
  }

  public static class intakeConstants {    
    public static final double kDefaultSpeed = 0.8;
    public static final int kPhotoSensorPortPWM = 0;

    public static final double kP = 0.00002;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class falonConst {
    public static final double kS = 0.25;
    public static final double kV = 0.12;
    public static final double kA = 0.01;
    public static final double kP = 0.11;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final int kFalconAccel = 100;
  }

  public static class motorPorts {
    public static final int rightMaster = 1;
    public static final int rightFollower = 2;
    public static final int leftMaster = 3;
    public static final int leftFollower = 4;
    public static final int intakeMotor = 9;
    public static final int feedMotor = 13;
    public static final int shooterMotor = 12;
    public static final int kTalonPort = 12;
  }

  public static class SwerveConst {
    public static final double[] frontLeftModXY = {-10.0, -10.0};
    public static final double[] frontRightModXY = {10.0, -10.0};
    public static final double[] rearLeftRModXY = {-10.0, 10.0};
    public static final double[] rearRightModXY = {10.0, 10.0};

    public static final int frontLeftModAzimuth = 2;
    public static final int frontLeftModDrive = 3;

    public static final int frontRightModAzimuth = 4;
    public static final int frontRightModDrive = 5;

    public static final int rearLeftRModAzimuth = 6;
    public static final int rearLeftRModDrive = 7;

    public static final int rearRightModAzimuth = 8;
    public static final int rearRightModDrive = 9;
  }

}
