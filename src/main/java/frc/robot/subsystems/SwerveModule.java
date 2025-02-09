// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

public class SwerveModule extends SubsystemBase {
  /** Creates a new Swerve Module Subsystem. */

  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private final SparkMaxConfig m_driveMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_turningMotorConfig = new SparkMaxConfig();

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;
  private final CANcoder m_CANcoder;

  private final SparkClosedLoopController m_driveClosedLoopController;
  //private final PIDController m_turningPIDController = new PIDController(SwerveConstants.turnGainP, SwerveConstants.turnGainI, SwerveConstants.turnGainD);
  private final SparkClosedLoopController m_turnClosedLoopController;

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
    m_driveMotorConfig.encoder
      .positionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor)
      .velocityConversionFactor(SwerveConstants.kDrivingEncoderVelocityFactor);
    m_driveMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(SwerveConstants.driveGainP, SwerveConstants.driveGainI, SwerveConstants.driveGainD);
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.configure(m_driveMotorConfig, null, null);

    m_driveMotorChannel = driveMotorChannel; //for debugging

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveClosedLoopController = m_driveMotor.getClosedLoopController();

    //setup turning motor info
    m_turningMotorConfig
      .inverted(SwerveConstants.kTurningEncoderInverted);
    m_turningMotorConfig.encoder
      .positionConversionFactor(SwerveConstants.kTurningEncoderPositionFactor)
      .velocityConversionFactor(SwerveConstants.kTurningEncoderVelocityFactor);
    m_turningMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(SwerveConstants.turnGainP, SwerveConstants.turnGainI, SwerveConstants.turnGainD);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.configure(m_turningMotorConfig, null, null);

    m_turnEncoder = m_turningMotor.getEncoder();
    m_turnClosedLoopController = m_turningMotor.getClosedLoopController();
    m_CANcoder = new CANcoder(turningEncoderChannel);
    //m_CANcoder.configure.setInverted();
    m_moduleEncoderAngularOffset = moduleEncoderAngularOffset*360;

    // Set the PID gains for the driving motor. 
    // May need to tune.
//    m_driveClosedLoopController.setFF(0);
//    m_driveClosedLoopController.setOutputRange(-1, 1); 

     // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    /*m_turningPIDController.setFF(0);
    m_turningPIDController.setOutputRange(SwerveConstants.kTurningMinOutput,
        SwerveConstants.kTurningMaxOutput);*/
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);


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

    //m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = getAngle();//new Rotation2d(m_CANcoder.getPosition().getValue());
    //m_driveEncoder.setPosition(0);
    resetEncoders();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    
    //var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());
 
    // Optimize the reference state to avoid spinning further than 90 degrees
    //SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    //state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromDegrees(m_moduleEncoderAngularOffset));
    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        getAngle());
        //new Rotation2d(m_CANcoder.getPosition().getValue()));

   
    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_driveClosedLoopController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turnClosedLoopController.setReference(optimizedDesiredState.angle.getDegrees(), SparkMax.ControlType.kPosition);


    // Calculate the turning motor output from the turning PID controller.
    //final double turnOutput = m_turningPIDController.calculate(wheelAngle(), optimizedDesiredState.angle.getDegrees());

    // Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(driveOutput);
    //m_turningMotor.set(turnOutput);
    
    m_desiredState = desiredState;
    
  }
     
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(),
        getAngle());
        //getCanCoder());
        //new Rotation2d(m_CANcoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
  }


  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
      m_driveEncoder.setPosition(0);
      m_turnEncoder.setPosition(wheelAngle());
      //m_turningEncoder.reset();
  }

  public double TurnOutput() {
    double turn = m_CANcoder.getAbsolutePosition().getValueAsDouble();//getCanCoder().getDegrees();//StatusSignal<Angle> turn = m_CANcoder.getPosition();
    return turn;//.getValueAsDouble();
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
    var angle = getCanCoder();
    double angleDeg = angle.getDegrees()-m_moduleEncoderAngularOffset;
    return angleDeg;
  }

  public double distance() {
    var distance = m_driveEncoder.getPosition();
    return distance;
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - m_moduleEncoderAngularOffset;
    //m_turnEncoder.setPosition(absolutePosition);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(m_CANcoder.getAbsolutePosition().getValueAsDouble()*360);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_turnEncoder.getPosition());
  }

  public double getTurnAngle() {
    return m_turnEncoder.getPosition();
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
