// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
//import frc.lib.util.CANCoderUtil;

public class SwerveModule extends SubsystemBase {
  /** Creates a new Swerve Module Subsystem. */

  //private Rotation2d lastAngle;
  //private Rotation2d angleOffSet;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;
  private final CANcoder m_CANcoder;

  private final SparkPIDController m_drivePIDController;
  private final SparkPIDController m_turnPIDController;

  private int m_driveMotorChannel; //for debugging

  private double m_moduleEncoderAngularOffset;
  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN input for the drive motor.
   * @param turningMotorChannel CAN input for the turning motor.
   * @param turningEncoderChannel CAN input for the turning encoder channel
   * 
   */
  public SwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    int turningEncoderChannel,
    double moduleEncoderAngularOffset) {

    //setup driving motor info
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotorChannel = driveMotorChannel; //for debugging
    m_driveEncoder = m_driveMotor.getEncoder();
    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setFeedbackDevice(m_driveEncoder);
    m_driveMotor.restoreFactoryDefaults();

    //setup turning motor info
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turnEncoder = m_turningMotor.getEncoder();
    m_turnPIDController = m_turningMotor.getPIDController();
    m_turnPIDController.setFeedbackDevice(m_turnEncoder);
    m_turningMotor.restoreFactoryDefaults();
    //m_turningMotor.burnFlash();


    m_CANcoder = new CANcoder(turningEncoderChannel);
    m_CANcoder.getConfigurator().apply(new CANcoderConfiguration());


    //resetToAbsolute();

    m_moduleEncoderAngularOffset = moduleEncoderAngularOffset;
    
    /* Speed up signals to an appropriate rate */
    //BaseStatusSignal.setUpdateFrequencyForAll(100, m_turningEncoder.getPosition(), m_turningEncoder.getVelocity());

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_driveEncoder.setPositionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor);
    m_driveEncoder.setVelocityConversionFactor(SwerveConstants.kDrivingEncoderVelocityFactor);

    m_turnEncoder.setPositionConversionFactor(SwerveConstants.kTurningEncoderPositionFactor);
    m_turnEncoder.setVelocityConversionFactor(SwerveConstants.kTurningEncoderVelocityFactor);

    //m_NEOturnEncoder.setInverted(SwerveConstants.kTurningEncoderInverted);

    // Set the PID gains for the driving motor. 
    // May need to tune.
    m_drivePIDController.setP(SwerveConstants.driveGainP);
    m_drivePIDController.setI(SwerveConstants.driveGainI);
    m_drivePIDController.setD(SwerveConstants.driveGainD);
    m_drivePIDController.setFF(SwerveConstants.driveFF);
    m_drivePIDController.setOutputRange(-1, 1); 

     // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turnPIDController.setP(SwerveConstants.turnGainP);
    m_turnPIDController.setI(SwerveConstants.turnGainI);
    m_turnPIDController.setD(SwerveConstants.turnGainD);
    m_turnPIDController.setFF(SwerveConstants.turnFF);

    // Set the PID wrapping for the controller
    m_turnPIDController.setPositionPIDWrappingEnabled(true);
    m_turnPIDController.setPositionPIDWrappingMinInput(0);
    m_turnPIDController.setPositionPIDWrappingMaxInput(90);
    //m_turnPIDController.setOutputRange(SwerveConstants.kTurningMinOutput, SwerveConstants.kTurningMaxOutput);



    //m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
    //m_driveEncoder.setPosition(0);
    resetEncoders();

    //lastAngle = getState().angle;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turnEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turnPIDController.setReference(optimizedDesiredState.angle.getDegrees(), CANSparkMax.ControlType.kPosition);

    //m_desiredState = desiredState;
  }

  public SwerveModulePosition getModulePosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        new Rotation2d(m_turnEncoder.getPosition()));
  }


  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
      m_driveEncoder.setPosition(0);
      m_turnEncoder.setPosition(-wheelAngle());
  }

  public double TurnOutput() {
    double turn = m_CANcoder.getAbsolutePosition().getValueAsDouble();
    //double turn = m_turnEncoder.getPosition();
    return turn;
  }

  public double DriveOutput() {
    double drive = m_driveEncoder.getVelocity();
    return drive;
  }

  public void DriveStop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  public double wheelAngle() {
    var angle = new Rotation2d(m_CANcoder.getAbsolutePosition().getValueAsDouble()*Math.PI*2); // CANcoder output is -0.5 to 0.5, must scale to radians by 2*PI
    //var angle = new Rotation2d(m_NEOturnEncoder.getPosition());
    double angleDeg = angle.getDegrees()-m_moduleEncoderAngularOffset;
    return angleDeg;
  }

  public double distance() {
    return m_driveEncoder.getPosition();
    //var distance = m_driveEncoder.getPosition();
    //return distance;
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - m_moduleEncoderAngularOffset*360;
    m_turnEncoder.setPosition(absolutePosition);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(m_CANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_turnEncoder.getPosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
