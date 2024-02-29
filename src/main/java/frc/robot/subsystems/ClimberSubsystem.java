// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;
import frc.robot.extensions.SparkMaxPIDTuner;

public class ClimberSubsystem extends SubsystemBase {
 
 private SendableCANSparkMax rightClimberMotor;
 private SendableCANSparkMax leftClimberMotor;

 private RelativeEncoder rightPositionEncoder;
 private RelativeEncoder leftPositionEncoder;


 private DigitalInput rightClimbHomeLimit; 
 private DigitalInput leftClimbHomeLimit; 
 private double rightClimberSpeed = 0;
 private double leftClimberSpeed = 0;

 private boolean locked;
 
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    rightClimberMotor = new SendableCANSparkMax(Constants.climberSubsystem.kRightClimberID, MotorType.kBrushless);
    leftClimberMotor = new SendableCANSparkMax(Constants.climberSubsystem.kLeftClimberID, MotorType.kBrushless);

    rightClimberMotor.restoreFactoryDefaults();
    leftClimberMotor.restoreFactoryDefaults();

    rightClimberMotor.setInverted(false);
    leftClimberMotor.setInverted(true);

    rightClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);

    rightPositionEncoder = rightClimberMotor.getEncoder();
    leftPositionEncoder = leftClimberMotor.getEncoder();
  
    // rightPositionEncoder.setInverted(Constants.climberSubsystem.kRightEncoderInverted);
    // leftPositionEncoder.setInverted(Constants.climberSubsystem.kLeftEncoderInverted);
    
    rightPositionEncoder.setPositionConversionFactor(Constants.climberSubsystem.kConversionFactor);
    leftPositionEncoder.setPositionConversionFactor(Constants.climberSubsystem.kConversionFactor);

    rightClimbHomeLimit = new DigitalInput(Constants.climberSubsystem.kRightHomeLimitport);
    leftClimbHomeLimit = new DigitalInput(Constants.climberSubsystem.kLeftHomeLimitport);

    locked = true;

    Shuffleboard.getTab("Climber").add("Right", rightClimberMotor);
    Shuffleboard.getTab("Climber").add("left", leftClimberMotor);

    Shuffleboard.getTab("Climber").add("leftLimit", leftClimbHomeLimit);
    Shuffleboard.getTab("Climber").add("RightLimit", rightClimbHomeLimit);

    // home();
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

  public void home(){ 
    setSpeed(Constants.climberSubsystem.kHomingspeed);
  }

  public double leftHeight(){
    return leftPositionEncoder.getPosition();
  }

  public double rigthHeight(){
    return rightPositionEncoder.getPosition();
  }

  public boolean isLeftHomed(){
    return leftClimbHomeLimit.get() == Constants.climberSubsystem.kHomePressed;
  }

  public boolean isRigthHomed(){
    return rightClimbHomeLimit.get() == Constants.climberSubsystem.kHomePressed;
  }

  public boolean isLocked(){
    return locked;
  }

  public void unlock(){
    locked = false;
  }

    public void lock(){
    locked = true;
  }

  @Override
  public void periodic() {
    double appliedRightSpeed = rightClimberSpeed;
    double appliedLeftSpeed = leftClimberSpeed;

    if(appliedRightSpeed > 0){ //moving up
      if(rightPositionEncoder.getPosition() >= Constants.climberSubsystem.kUpperPosition){
        appliedRightSpeed = 0;
      }
    }
    else if(appliedRightSpeed < 0){ //moving down
      if(rightClimbHomeLimit.get() == Constants.climberSubsystem.kHomePressed){
        appliedRightSpeed = 0;
        rightPositionEncoder.setPosition(Constants.climberSubsystem.kHomingPosition - Constants.climberSubsystem.kHomingOffset);
      } else if(rightPositionEncoder.getPosition() < Constants.climberSubsystem.kLowerCautionPosition) {
        appliedRightSpeed = appliedRightSpeed * Constants.climberSubsystem.kCautionSpeedMultiplier;
      }
    }
    
    if(appliedLeftSpeed > 0){ //moving up
      if(leftPositionEncoder.getPosition() >= Constants.climberSubsystem.kUpperPosition){
        appliedLeftSpeed = 0;
      }
    }
    else if(appliedLeftSpeed < 0){ //moving down
      if(leftClimbHomeLimit.get() == Constants.climberSubsystem.kHomePressed){
        appliedLeftSpeed = 0;
        leftPositionEncoder.setPosition(Constants.climberSubsystem.kHomingPosition - Constants.climberSubsystem.kHomingOffset);
      } else if(leftPositionEncoder.getPosition() < Constants.climberSubsystem.kLowerCautionPosition) {
        appliedLeftSpeed = appliedLeftSpeed * Constants.climberSubsystem.kCautionSpeedMultiplier;
      }
    }

    rightClimberMotor.set(appliedRightSpeed);
    leftClimberMotor.set(appliedLeftSpeed);
    
  }

  @Override
  public void simulationPeriodic() {
  }
}
