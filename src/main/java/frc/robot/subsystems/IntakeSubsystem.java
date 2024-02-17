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
    topWheelMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kTopWheelMotorPortID, MotorType.kBrushless);
    bottomStarMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kBottomStarMotorPortID, MotorType.kBrushless);
    
    safeMode = true;
    homing = true;

    intakeHomeLimit = new DigitalInput(Constants.intakeSubsystem.kHomeLimitID);

    positionMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kPositionMotorPortID, MotorType.kBrushless);
    positionEncoder = positionMotor.getEncoder();
    targetPosition = Constants.intakeSubsystem.kTargetPositionUp;
    positionPID = positionMotor.getPIDController();

    positionLimiter = new SlewRateLimiter(
      Constants.intakeSubsystem.kPositionRateLimit,
     -Constants.intakeSubsystem.kPositionRateLimit,
      Constants.intakeSubsystem.kPositionInitialValue);
    safeModeLimiter = new SlewRateLimiter(
      Constants.intakeSubsystem.kSafePositionRateLimit,
     -Constants.intakeSubsystem.kSafePositionRateLimit,
      Constants.intakeSubsystem.kSafePositionInitialValue);
  }

  //TODO: have a scaler so that you can achive a ratio between the two motors, one wheel has a larger diameter than the other and will turn at a diffrent speed becuse of such.
  public void ejectNote(){ 
    topWheelMotor.set(-Constants.intakeSubsystem.kNoteMotorSpeed);
    bottomStarMotor.set(-Constants.intakeSubsystem.kNoteMotorSpeed);
  }

  public void injectNote(){
    topWheelMotor.set(Constants.intakeSubsystem.kNoteMotorSpeed);
    bottomStarMotor.set(Constants.intakeSubsystem.kNoteMotorSpeed);
  }
  
  public void stopNoteMotors(){
    topWheelMotor.set(Constants.intakeSubsystem.kStopNoteMotors);
    bottomStarMotor.set(Constants.intakeSubsystem.kStopNoteMotors);
  }

  public void positionUp(){
    //intakePosition = IntakePositions.Up;
    targetPosition = Constants.intakeSubsystem.kTargetPositionUp;
  }
  
  public void positionDown(){
    //intakePosition = IntakePositions.Down;
    targetPosition = Constants.intakeSubsystem.kTargetPositionDown;
  }

  public void setHoming(boolean homingState){
    setSafeMode(true);
    homing = homingState;
  }

  //TODO: need a command for exiting safe mode?
  public void setSafeMode(boolean safeModeState){
    safeMode = safeModeState;
  }


  @Override
  public void periodic() {
    if(homing){
      targetPosition = positionEncoder.getPosition() - Constants.intakeSubsystem.kHomingVel;

      if(intakeHomeLimit.get() == Constants.intakeSubsystem.kHomeLimitPressed){
        homing = false;

        //TODO: needs to set refrence of pid controller to 0 veloctity to, setings motor to 0 will not stop pid controller from acting on it.
        positionMotor.set(0);   
        positionEncoder.setPosition(Constants.intakeSubsystem.kHomingPosition - Constants.intakeSubsystem.kHomingOffset);
      }
    }
  
    if(safeMode) {
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
