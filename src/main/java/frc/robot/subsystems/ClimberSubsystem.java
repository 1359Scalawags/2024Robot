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
 
 private SendableCANSparkMax rightClimberMotor;
 private SendableCANSparkMax leftClimberMotor;

 private RelativeEncoder rightPositionEncoder;
 private RelativeEncoder leftPositionEncoder;


 private DigitalInput rightClimbHomeLimit; 
 private DigitalInput leftClimbHomeLimit; 
 private double climberSpeed = 0;
 private boolean homing;
 
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    rightClimberMotor = new SendableCANSparkMax(Constants.ClimberSubsystem.kRightClimberID, MotorType.kBrushless);
    leftClimberMotor = new SendableCANSparkMax(Constants.ClimberSubsystem.kLeftClimberID,MotorType.kBrushless);

    rightPositionEncoder = rightClimberMotor.getEncoder();
    leftPositionEncoder = leftClimberMotor.getEncoder();

    rightClimbHomeLimit = new DigitalInput(Constants.ClimberSubsystem.kRightHomeLimitport);
    leftClimbHomeLimit = new DigitalInput(Constants.ClimberSubsystem.kLeftHomeLimitport);
    homing = true;
  }
  public void setSpeed (double speed){
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
    if (homing) {
      climberSpeed = Constants.ClimberSubsystem.kHomingspeed;
    }

    if(climberSpeed>0){
      if(rightPositionEncoder.getPosition() >= Constants.ClimberSubsystem.kUpperPosition){
        leftClimberMotor.set(0);
        climberSpeed = 0;
      }
      else{
        leftClimberMotor.set(climberSpeed);
      }
    }
    else if(climberSpeed<0){
      if(leftClimbHomeLimit.get()){
        leftClimberMotor.set(0);
      }
      else{
        leftClimberMotor.set(climberSpeed);
      }

      if(rightClimbHomeLimit.get()){
        rightClimberMotor.set(0);
      }
      else{
        rightClimberMotor.set(climberSpeed);
      }
      
      if(rightClimbHomeLimit.get() && leftClimbHomeLimit.get()){
        climberSpeed = 0;
        homing = false;
      }

    }
    else {
      leftClimberMotor.set(climberSpeed);
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
