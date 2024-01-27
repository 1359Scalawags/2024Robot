// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;

public class ClimberSubsystem extends SubsystemBase {
 
 private SendableCANSparkMax climberMotor;
 
 private RelativeEncoder positionEncoder;

 private DigitalInput climberLowerlimit; 
 private DigitalInput climberUperlimit; 
 private double climberSpeed = 0;
 
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new SendableCANSparkMax(Constants.ClimberSubsystem.kClimberMotorPort,MotorType.kBrushless);
    positionEncoder = climberMotor.getEncoder();

    climberUperlimit = new DigitalInput(Constants.ClimberSubsystem.kClimberUperlimitswitchport);
    climberLowerlimit = new DigitalInput(Constants.ClimberSubsystem.kClimberLowerlimitswitchport);

  }
  private void setSpeed (double speed){
   climberSpeed = speed;
   
  }

  public void extendArm(){
    setSpeed(Constants.ClimberSubsystem.kClimberExtendRate);
  }
  public void retractArm(){
    setSpeed(-Constants.ClimberSubsystem.kClimberExtendRate);
  }
public void stopArm(){
    setSpeed(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    if(climberSpeed>0){
      if(climberUperlimit.get()){
        climberMotor.set(0);
        climberSpeed = 0;
      }
      else{
        climberMotor.set(climberSpeed);
      }
    }
    else if(climberSpeed<0){
      if(climberLowerlimit.get()){
        climberMotor.set(0);
        climberSpeed = 0;
      }
      else{
        climberMotor.set(climberSpeed);
      }
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
