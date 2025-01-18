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

  private Rotation2d lastAngle;
  //private Rotation2d angleOffSet;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_turnEncoder;
  private final RelativeEncoder m_NEOturnEncoder;

  private final SparkPIDController m_drivePIDController;
  //private final PIDController m_turnPIDController = new PIDController(SwerveConstants.turnGainP, SwerveConstants.turnGainI, SwerveConstants.turnGainD);
  private final SparkPIDController m_turnPIDController;

  private int m_driveMotorChannel; //for debugging

  private double m_moduleEncoderAngularOffset;
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
    m_turnEncoder = new CANcoder(turningEncoderChannel);
    m_NEOturnEncoder = m_turningMotor.getEncoder();
    m_turnPIDController = m_turningMotor.getPIDController();
    m_turnPIDController.setFeedbackDevice(m_NEOturnEncoder);
    //m_turningPIDController.setFeedbackDevice(m_turningEncoder);
    m_turningMotor.restoreFactoryDefaults();
    resetToAbsolute();

    m_moduleEncoderAngularOffset = moduleEncoderAngularOffset;
    /* Configure CANcoder */
    //var toApply = new CANcoderConfiguration();

    /* User can change the configs if they want, or leave it empty for factory-default */
    //m_turningEncoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    //BaseStatusSignal.setUpdateFrequencyForAll(100, m_turningEncoder.getPosition(), m_turningEncoder.getVelocity());

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_driveEncoder.setPositionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor);
    m_driveEncoder.setVelocityConversionFactor(SwerveConstants.kDrivingEncoderVelocityFactor);

    m_NEOturnEncoder.setPositionConversionFactor(SwerveConstants.kTurningEncoderPositionFactor);
    m_NEOturnEncoder.setVelocityConversionFactor(SwerveConstants.kTurningEncoderVelocityFactor);

    //m_NEOturnEncoder.setInverted(SwerveConstants.kTurningEncoderInverted);

    // Set the PID gains for the driving motor. 
    // May need to tune.
    m_drivePIDController.setP(SwerveConstants.driveGainP);
    m_drivePIDController.setI(SwerveConstants.driveGainI);
    m_drivePIDController.setD(SwerveConstants.driveGainD);
    m_drivePIDController.setFF(0);
    m_drivePIDController.setOutputRange(-1, 1); 

     // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turnPIDController.setP(SwerveConstants.turnGainP);
    m_turnPIDController.setI(SwerveConstants.turnGainI);
    m_turnPIDController.setD(SwerveConstants.turnGainD);
    m_turnPIDController.setFF(0);
    m_turnPIDController.setOutputRange(SwerveConstants.kTurningMinOutput,
        SwerveConstants.kTurningMaxOutput);
    //m_turnPIDController.enableContinuousInput(-Math.PI, Math.PI);


    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / SwerveConstants.kAngleEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    /*m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(SwerveConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(SwerveConstants.kTurningEncoderPositionPIDMaxInput);
    */

    m_moduleEncoderAngularOffset = moduleEncoderAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition().getValueAsDouble());
    m_driveEncoder.setPosition(0);

    lastAngle = getState().angle;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    
    var encoderRotation = new Rotation2d(m_NEOturnEncoder.getPosition());
 
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    
    //SwerveModuleState correctedDesiredState = new SwerveModuleState();
    //correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    //correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    // Optimize the reference state to avoid spinning further than 90 degrees.
    //SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        //new Rotation2d(m_turnEncoder.getPosition().getValueAsDouble()));

   
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivePIDController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turnPIDController.setReference(desiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);


    // Calculate the turning motor output from the turning PID controller.
    //final double turnOutput =
        //m_turningPIDController.calculate(m_turningEncoder.getPosition().getValueAsDouble(), optimizedDesiredState.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(driveOutput);
   // m_turningMotor.set(turnOutput);
    
    m_desiredState = desiredState;
    
  }
     
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        new Rotation2d(m_turnEncoder.getPosition().getValueAsDouble() - m_moduleEncoderAngularOffset));
  }


  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
      m_driveEncoder.setPosition(0);
      //m_turningEncoder.reset();
  }

  public double TurnOutput() {
    double turn = m_turnEncoder.getAbsolutePosition().getValueAsDouble();
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
    var angle = new Rotation2d(m_NEOturnEncoder.getPosition());
    double angleDeg = angle.getDegrees();
    return angleDeg;
  }

  public double distance() {
    var distance = m_driveEncoder.getPosition();
    return distance;
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - m_moduleEncoderAngularOffset;
    m_NEOturnEncoder.setPosition(absolutePosition);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(m_turnEncoder.getAbsolutePosition().getValueAsDouble());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_NEOturnEncoder.getPosition());
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
