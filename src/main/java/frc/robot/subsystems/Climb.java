package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.Spark;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.motorConstants;


public class Climb extends SubsystemBase {
  public Climb(){}
   public Spark leftClimb = new CANSparkMax(motorConstants.CmotorL);
   public Spark rightClimb = new CANSparkMax(motorConstants.CmotorR);

  public void climb(){

  }
  public void letGo(){

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
