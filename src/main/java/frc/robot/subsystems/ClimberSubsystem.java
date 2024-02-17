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
 private double rightClimberSpeed = 0;
 private double leftClimberSpeed = 0;
 
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    rightClimberMotor = new SendableCANSparkMax(Constants.climberSubsystem.kRightClimberID, MotorType.kBrushless);
    leftClimberMotor = new SendableCANSparkMax(Constants.climberSubsystem.kLeftClimberID, MotorType.kBrushless);

    rightPositionEncoder = rightClimberMotor.getEncoder();
    leftPositionEncoder = leftClimberMotor.getEncoder();
    
    //TODO: Reset factory defaults
    //TODO: constant to invert right and left position encoders?
    //TODO: set position encoders conversion factors

    rightClimbHomeLimit = new DigitalInput(Constants.climberSubsystem.kRightHomeLimitport);
    leftClimbHomeLimit = new DigitalInput(Constants.climberSubsystem.kLeftHomeLimitport);

    //TODO: Reverse Motor Controller
  }

  public final void setSpeed (double speed){
    rightClimberSpeed = speed;
    leftClimberSpeed = speed;
  }

  public void extendArm(){
    setSpeed(Constants.climberSubsystem.kClimberExtendRate);
  }

  public void retractArm(){
    setSpeed(-Constants.climberSubsystem.kClimberExtendRate);
  }
  public void stopArm(){
    setSpeed(0);
  }

  public void home(){ //TODO: make command that homes, should always run at start of robot.
    setSpeed(-Constants.climberSubsystem.kHomingspeed);
  }

  @Override
  public void periodic() {
    if(rightClimberSpeed > 0){ //moving up
      if(rightPositionEncoder.getPosition() >= Constants.climberSubsystem.kUpperPosition){
        rightClimberSpeed = 0;
      }
    }
    else if(rightClimberSpeed < 0){ //moving down
      if(rightClimbHomeLimit.get() == Constants.climberSubsystem.kHomePressed){
        rightClimberSpeed = 0;
        rightPositionEncoder.setPosition(Constants.climberSubsystem.kHomingPosition - Constants.climberSubsystem.kHomingOffset);
      }
    }
    rightClimberMotor.set(rightClimberSpeed);

    if(leftClimberSpeed > 0){ //moving up
      if(leftPositionEncoder.getPosition() >= Constants.climberSubsystem.kUpperPosition){
        leftClimberSpeed = 0;
      }
    }
    else if(leftClimberSpeed < 0){ //moving down
      if(leftClimbHomeLimit.get() == Constants.climberSubsystem.kHomePressed){
        leftClimberSpeed = 0;
        leftPositionEncoder.setPosition(Constants.climberSubsystem.kHomingPosition - Constants.climberSubsystem.kHomingOffset);
      }
    }
    leftClimberMotor.set(leftClimberSpeed);
  }

  @Override
  public void simulationPeriodic() {
  }
}
