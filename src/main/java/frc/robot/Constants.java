// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final double gearRatio1st = 14/50;
    public static final double gearRatio2nd = 27/17;
    public static final double gearRatio3rd = 15/45;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // meters

    public static final double kDrivingEncoderPositionFactor = kWheelDiameterMeters*Math.PI/(gearRatio1st*gearRatio2nd*gearRatio3rd);
    public static final double kDrivingEncoderVelocityFactor = (kDrivingEncoderPositionFactor/60); // meters per second

    public static final double kturnGearRatio = 150/7;
    public static final double kTurningEncoderPositionFactor = 1*(kturnGearRatio)/1.25; // position in degrees  //(2 * Math.PI) / kturnGearRatio; //position in radians
    public static final double kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor/60; // meters per second
    
    public static final double driveGainP = 0.00002;
    public static final double driveGainI = 0.000001;
    public static final double driveGainD = 0.0015;
    public static final double driveFF = 1/473;

    public static final double turnGainP = 0.00075;//00075;
    public static final double turnGainI = 0.000001;
    public static final double turnGainD = 0.025;//.003;
    public static final double turnFF = 0;

    public static final double kAngleEncoderResolution = 42;
    public static final boolean kTurningEncoderInverted = true;
    public static final double kTurningMinOutput = -0.1;
    public static final double kTurningMaxOutput = 0.1;
    public static final double kTurningEncoderPositionPIDMinInput = -1;
    public static final double kTurningEncoderPositionPIDMaxInput = 1;
  }

  public static class DriveConstants {
    public static final double robotWidth = Units.inchesToMeters(37);
    public static final double robotLength = Units.inchesToMeters(37);
    public static final double WheelYdist = robotLength*0.5;
    public static final double WheelXdist = robotWidth*0.5;

    // module angluar offset in Degrees
    public static final double kFrontLeftChassisAngularOffset = 90; 
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 270;
    public static final double kBackRightChassisAngularOffset = 180;

    // encoder angluar offset in Degrees
    public static final double kFrontLeftModuleAngularOffset = -0.001465*360;//Math.PI*2; 
    public static final double kFrontRightModuleAngularOffset = -0.466046*360;//Math.PI*2;
    public static final double kBackLeftModuleAngularOffset = -0.070313*360;//Math.PI*2;
    public static final double kBackRightModuleAngularOffset = 0.182617*360;//Math.PI*2;
 
    public static final double kChassisAngularOffset = 0;

    /*public static final SwerveDriveKinematics kswerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(WheelXdist, WheelYdist),
      new Translation2d(WheelXdist, -WheelYdist),
      new Translation2d(-WheelXdist, WheelYdist),
      new Translation2d(-WheelXdist, -WheelYdist));*/

    public static final double kMaxSpeed = 1000;//5000; // speed in mm/s
    public static final double kMaxAngularSpeed = 0.5;
    public static final double kDriveDeadband = 0.05;
    public static final double kDriveDeadbandZ = 0.1;
    public static final boolean kTeleField = false;
  }

public static class ControlSystem {
    // Driving motor CAN IDs
    public static final int kLeftFrontDrive = 2;
    public static final int kLeftBackDrive = 4;
    public static final int kRightFrontDrive = 1;
    public static final int kRightBackDrive = 3;
    // Turning motors
    public static final int kLeftFrontTurn = 10;
    public static final int kLeftBackTurn = 9;
    public static final int kRightFrontTurn = 11;
    public static final int kRightBackTurn = 12;
    // CANCoder Can IDs for tunring encoders
    public static final int kLFturn = 13;
    public static final int kLBturn = 16;
    public static final int kRFturn = 14;
    public static final int kRBturn = 15;
  }
}
