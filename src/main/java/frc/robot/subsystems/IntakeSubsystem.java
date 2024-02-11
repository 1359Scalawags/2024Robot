// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;


public class IntakeSubsystem extends SubsystemBase {
 
  // enum IntakePositions{
  //   Up,
  //   Down,
  //   NotMoving
  // }

  //private IntakePositions intakePosition;
  private SendableCANSparkMax topWheelMotor;
  private SendableCANSparkMax bottomStarMotor;
  private SendableCANSparkMax positionMotor;
  private RelativeEncoder positionEncoder;
 
  private double targetPosition;
  private SparkPIDController positionPID;
  private SlewRateLimiter positionLimiter;
  private SlewRateLimiter safeModeLimiter;

  private DigitalInput intakeHomeLimit;
  private boolean homing;

  private boolean safeMode;
 
 
 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    //intakePosition = IntakePositions.Up;
    topWheelMotor = new SendableCANSparkMax(Constants.Intake.kTopWheelMotorPortID, MotorType.kBrushless);
    bottomStarMotor = new SendableCANSparkMax(Constants.Intake.kBottomStarMotorPortID, MotorType.kBrushless);
    
    safeMode = true;
    homing = true;

    intakeHomeLimit = new DigitalInput(Constants.Intake.kHomeLimitID);

    positionMotor = new SendableCANSparkMax(Constants.Intake.kPositionMotorPortID, MotorType.kBrushless);
    positionEncoder = positionMotor.getEncoder();
    targetPosition = Constants.Intake.kTargetPositionUp;
    positionPID = positionMotor.getPIDController();

    positionLimiter = new SlewRateLimiter(
      Constants.Intake.kPositionRateLimit,
     -Constants.Intake.kPositionRateLimit,
      Constants.Intake.kPositionInitialValue);
    safeModeLimiter = new SlewRateLimiter(
      Constants.Intake.kSafePositionRateLimit,
     -Constants.Intake.kSafePositionRateLimit,
      Constants.Intake.kSafePositionInitialValue);
  }

  public void ejectNote(){
    topWheelMotor.set(-Constants.Intake.kNoteMotorSpeed);
    bottomStarMotor.set(-Constants.Intake.kNoteMotorSpeed);
  }

  public void injectNote(){
    topWheelMotor.set(Constants.Intake.kNoteMotorSpeed);
    bottomStarMotor.set(Constants.Intake.kNoteMotorSpeed);
  }
  
  public void stopNoteMotors(){
    topWheelMotor.set(Constants.Intake.kStopNoteMotors);
    bottomStarMotor.set(Constants.Intake.kStopNoteMotors);
  }

  public void positionUp(){
    //intakePosition = IntakePositions.Up;
    targetPosition = Constants.Intake.kTargetPositionUp;
  }
  
  public void positionDown(){
    //intakePosition = IntakePositions.Down;
    targetPosition = Constants.Intake.kTargetPositionDown;
  }

  @Override
  public void periodic() {
    if(homing){
      if(intakeHomeLimit.get() == Constants.Intake.kHomeLimitPressed){
        homing = false;
        positionPID.setReference(0, ControlType.kVelocity);
        positionEncoder.setPosition(Constants.Intake.kHomingPosition);
      }
      else {
        positionPID.setReference(Constants.Intake.kHomingVel, ControlType.kVelocity);
      }
    }
    else if(safeMode) {
      double tempTarget = safeModeLimiter.calculate(targetPosition);
      positionPID.setReference(tempTarget, ControlType.kPosition);
    } else {
      double tempTarget = positionLimiter.calculate(targetPosition);
      positionPID.setReference(tempTarget, ControlType.kPosition);
    }




  //   if(intakePosition == IntakePositions.Up){
  //     if(positionEncoder.getPosition() < Constants.Intake.kMaxIntakePosition){
  //       positionMotor.set(Constants.Intake.kPositionMotorupSpeed);
  //     } else{
  //       positionMotor.set(0);
  //     }
  //   } else {
  //     if(positionEncoder.getPosition() > Constants.Intake.kMinIntakePosition){
  //       positionMotor.set(Constants.Intake.kPositionMotorDownSpeed);
  //     } else{
  //       positionMotor.set(0);
  //     }
  //   }
   }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
