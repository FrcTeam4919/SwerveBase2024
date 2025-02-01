// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControlSystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class DriveTrain extends SubsystemBase {
  /** Creates a new Drive Train Subsystem. */

  private final Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.WheelXdist, DriveConstants.WheelYdist);
  private final Translation2d m_frontRightLocation = new Translation2d(DriveConstants.WheelXdist, -DriveConstants.WheelYdist);
  private final Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.WheelXdist, DriveConstants.WheelYdist);
  private final Translation2d m_backRightLocation = new Translation2d(-DriveConstants.WheelXdist, -DriveConstants.WheelYdist);

  private final SwerveModule m_frontLeft= new SwerveModule(
    ControlSystem.kLeftFrontDrive,
    ControlSystem.kLeftFrontTurn, 
    ControlSystem.kLFturn, 
    DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
    ControlSystem.kRightFrontDrive,
    ControlSystem.kRightFrontTurn, 
    ControlSystem.kRFturn,
    DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_backLeft = new SwerveModule(
    ControlSystem.kLeftBackDrive,
    ControlSystem.kLeftBackTurn, 
    ControlSystem.kLBturn,
    DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_backRight = new SwerveModule(
    ControlSystem.kRightBackDrive,
    ControlSystem.kRightBackTurn, 
    ControlSystem.kRBturn,
    DriveConstants.kBackRightChassisAngularOffset);


  //private final ADIS16448_IMU m_imu = new ADIS16448_IMU();
  private final AHRS m_imu = new AHRS(SPI.Port.kMXP);
  
    /***********************************************************************
     * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
     * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
     * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * VMX-pi: - Communication via USB. - See
     * https://vmx-pi.kauailabs.com/installation/roborio-installation/
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    
  

  private final SwerveDriveKinematics m_kinematics =
    new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    
        
  private final SwerveDriveOdometry m_odometry =
    new SwerveDriveOdometry(
      m_kinematics,
      Rotation2d.fromDegrees(m_imu.getAngle()),
      new SwerveModulePosition[] {
        m_frontLeft.getModulePosition(),
        m_frontRight.getModulePosition(),
        m_backLeft.getModulePosition(),
        m_backRight.getModulePosition()
      });

  public DriveTrain() {
    try {
      //m_imu.calibrate();
      m_imu.reset();
    }
    catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MSP: " + ex.getMessage(), true);
    }
  }
   
  @Override
  public void periodic() {
    // update odometry
    m_odometry.update(
        Rotation2d.fromDegrees(m_imu.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getModulePosition(),
            m_frontRight.getModulePosition(),
            m_backLeft.getModulePosition(),
            m_backRight.getModulePosition()
        });

    // Put values to SmartDashboard 
    SmartDashboard.putNumber("Front Left Drive Speed", DriveVelFL());
    SmartDashboard.putNumber("Front Right Drive Speed", DriveVelFR());
    SmartDashboard.putNumber("Back Left Drive Speed", DriveVelBL());
    SmartDashboard.putNumber("Back Right Drive Speed", DriveVelBR());

    //Display Odometry IMU angle
    SmartDashboard.putNumber("Odometry Angle", getOdometryAngle());
    
    //Display Kinematics
    SmartDashboard.putNumber("Front Left Encoder Pos", TurnPosFL());
    SmartDashboard.putNumber("Back Left Encoder Pos", TurnPosBL());
    SmartDashboard.putNumber("Front Right Encoder Pos", TurnPosFR());
    SmartDashboard.putNumber("Back Right Encoder Pos", TurnPosBR());

    //Display Wheel orientations
     SmartDashboard.putNumber("FL Wheel Angle", wheelAngleFL());
     SmartDashboard.putNumber("FR Wheel Angle", wheelAngleFR());
     SmartDashboard.putNumber("BL Wheel Angle", wheelAngleBL());
     SmartDashboard.putNumber("BR Wheel Angle", wheelAngleBR());

    //Display Wheel orientations of NEOs
     SmartDashboard.putNumber("FL Wheel Angle NEO", wheelAngleNEOFL());
     SmartDashboard.putNumber("FR Wheel Angle NEO", wheelAngleNEOFR());
     SmartDashboard.putNumber("BL Wheel Angle NEO", wheelAngleNEOBL());
     SmartDashboard.putNumber("BR Wheel Angle NEO", wheelAngleNEOBR());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_imu.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getModulePosition(),
            m_frontRight.getModulePosition(),
            m_backLeft.getModulePosition(),
            m_backRight.getModulePosition()
        },
        pose);
  }

  public final double getOdometryAngle() {
    //System.out.printf("Odo Angle Call %f\n", m_imu.getAngle());
    double iMUAngle = m_imu.getAngle();
    return iMUAngle;
  } 
  
  /**
   * Method to drive the robot using the Joystick.
   * 
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param zRot Angular rotation of the robot by twisting the Joystick.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double zRot, boolean fieldRelative) {
    
    double xSpeedCommanded = xSpeed;
    double ySpeedCommanded = ySpeed;
    double m_currentRotation = zRot;


    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeed;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeed;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    
    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Z Rot ", zRot);
    SmartDashboard.putBoolean("Field Relative ", fieldRelative);
    if(xSpeed + ySpeed != 0) {System.out.printf("Original: Field %b, x=%f, y=%f, rot=%f\n", fieldRelative, xSpeed, ySpeed, zRot);}
    //if(xSpeed + ySpeed != 0) {System.out.printf("Delivered: Field %b, x=%f, y=%f, rot=%f\n", fieldRelative, xSpeedDelivered, ySpeedDelivered, rotDelivered);}

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
//            ? ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(m_imu.getAngle()))
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_imu.getAngle()))
//            : new ChassisSpeeds(20, 0, 0));
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    //ChassisSpeeds Speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    //ChassisSpeeds Speeds = new ChassisSpeeds(0, 0.2, 0);

    //SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(Speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
        
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]); 
      
    System.out.printf("Module state Vel %f", swerveModuleStates[3].speedMetersPerSecond);
    System.out.printf("Module state drive %f", m_backRight.getAngle().getDegrees());
    System.out.printf("Module state 0 %f\n", swerveModuleStates[3].angle.getRadians());
  }
    
   

  public void resetEncoders() {
    //m_frontLeft.resetEncoders();
    //m_backLeft.resetEncoders();
    //m_frontRight.resetEncoders();
    //m_backRight.resetEncoders();
  }

  // Measure turing encoder counts
  public double TurnPosFR() {
    double turningOut = m_frontRight.TurnOutput();
    return turningOut;
  }
  public double TurnPosFL() {
    double turningOut = m_frontLeft.TurnOutput();
    return turningOut;
  }
  public double TurnPosBR() {
    double turningOut = m_backRight.TurnOutput();
    return turningOut;
  }
  public double TurnPosBL() {
    double turningOut = m_backLeft.TurnOutput();
    return turningOut;
  }

  // Measure driving wheel speeds
  public double DriveVelFL() {
    double driveVelFL = m_frontLeft.DriveOutput();
    return driveVelFL;
  }
  public double DriveVelFR() {
    double driveVelFR = m_frontRight.DriveOutput();
    return driveVelFR;
  }
  public double DriveVelBL() {
    double driveVelBL = m_backLeft.DriveOutput();
    return driveVelBL;
  }
  public double DriveVelBR() {
    double driveVelBR = m_backRight.DriveOutput();
    return driveVelBR;
  }

  // Calculate wheel angles
  public double wheelAngleFL() {
    double angle = m_frontLeft.wheelAngle();
    return angle;
  }
  public double wheelAngleFR() {
    double angle = m_frontRight.wheelAngle();
    return angle;
  }
  public double wheelAngleBL() {
    double angle = m_backLeft.wheelAngle();
    return angle;
  }
  public double wheelAngleBR() {
    double angle = m_backRight.wheelAngle();
    return angle;
  }

  // Calculate wheel angles
  public double wheelAngleNEOFL() {
    double angle = m_frontLeft.getAngle().getDegrees();
    return angle;
  }
  public double wheelAngleNEOFR() {
    double angle = m_frontRight.getAngle().getDegrees();
    return angle;
  }
  public double wheelAngleNEOBL() {
    double angle = m_backLeft.getAngle().getDegrees();
    return angle;
  }
  public double wheelAngleNEOBR() {
    double angle = m_backRight.getAngle().getDegrees();
    return angle;
  }

  public double Distance() {
    var Distance = m_backLeft.distance();
    return Distance;
  }

/*
  public void DriveForward() {
    m_backLeft.DriveForward();
    m_frontLeft.DriveForward();
    m_backRight.DriveForward();
    m_frontRight.DriveForward();
  }
*/
  public void DriveStop() {
    m_backLeft.DriveStop();
    m_frontLeft.DriveStop();
    m_backRight.DriveStop();
    m_frontRight.DriveStop();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
