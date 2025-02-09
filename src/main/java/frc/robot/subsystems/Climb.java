package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.motorConstants;


public class Climb extends SubsystemBase {
  public Climb(){}
  
   private SparkMax leftClimb = new SparkMax(motorConstants.CmotorL, MotorType.kBrushless);
   private SparkMax rightClimb = new SparkMax(motorConstants.CmotorR, MotorType.kBrushless);
   //rightClimb.setInverted(true);
   // private Encoder CLE = new Encoder(0,1,false,Encoder.RelativeEncoder.k2X);
  public void climb(){
   leftClimb.set(0.5);
   rightClimb.set(0.5);
  }
  public void LetGo(){
    leftClimb.set(-0.5);
    rightClimb.set(-0.5);
  }
  //@Override
  public void stop(){
    leftClimb.set(0);
   rightClimb.set(0);
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
