// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.extensions.SendableCANSparkMax;


public class IntakeSubsystem extends SubsystemBase {
 
  // enum IntakePositions{
  //   Up,
  //   Down,
  //   NotMoving
  // }

  //private IntakePositions intakePosition;
  private SendableCANSparkMax topSushiMotor;
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
    //intakePosition = IntakePositions.Up;S
    topSushiMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kTopWheelMotorPortID, MotorType.kBrushless);
    bottomStarMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kBottomStarMotorPortID, MotorType.kBrushless);
    
    safeMode = true;
    setHoming(true);

    topSushiMotor.setInverted(false);
    bottomStarMotor.setInverted(false);

    topSushiMotor.setIdleMode(IdleMode.kBrake);
    bottomStarMotor.setIdleMode(IdleMode.kBrake);



    intakeHomeLimit = new DigitalInput(Constants.intakeSubsystem.kHomeLimitID);

    positionMotor = new SendableCANSparkMax(Constants.intakeSubsystem.kPositionMotorPortID, MotorType.kBrushless);

    positionMotor.restoreFactoryDefaults();
    positionMotor.setInverted(true);
    positionMotor.setIdleMode(IdleMode.kBrake);

    positionEncoder = positionMotor.getEncoder();
    positionEncoder.setPositionConversionFactor(Constants.intakeSubsystem.kIntakeConversionFactor);
    targetPosition = Constants.intakeSubsystem.kTargetPositionUp;
    positionPID = positionMotor.getPIDController();
    positionPID.setP(Constants.intakeSubsystem.kIntakeP);
    positionPID.setI(Constants.intakeSubsystem.kIntakeI);
    positionPID.setD(Constants.intakeSubsystem.kIntakeD);




    positionLimiter = new SlewRateLimiter(
      Constants.intakeSubsystem.kPositionRateLimit,
     -Constants.intakeSubsystem.kPositionRateLimit,
      Constants.intakeSubsystem.kPositionInitialValue);
    safeModeLimiter = new SlewRateLimiter(
      Constants.intakeSubsystem.kSafePositionRateLimit,
     -Constants.intakeSubsystem.kSafePositionRateLimit,
      Constants.intakeSubsystem.kSafePositionInitialValue);

    //Shuffleboard.getTab("LiveWindow").add(positionMotor);
    Shuffleboard.getTab("Intake").add("Position", positionMotor);
    Shuffleboard.getTab("Intake").add("Position Limitswitch", intakeHomeLimit);
  }

  public void ejectNote(){ 
    topSushiMotor.set(-Constants.intakeSubsystem.kSushiMotorSpeed);
    bottomStarMotor.set(-Constants.intakeSubsystem.kStarMotorSpeed);
  }

  public void injectNote(){
    topSushiMotor.set(Constants.intakeSubsystem.kSushiMotorSpeed);
    bottomStarMotor.set(Constants.intakeSubsystem.kStarMotorSpeed);
  }
  
  public void stopNoteMotors(){
    topSushiMotor.set(Constants.intakeSubsystem.kStopNoteMotors);
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
    targetPosition = -180;
  }

  //TODO: need a command for exiting safe mode?
  public void setSafeMode(boolean safeModeState){
    safeMode = safeModeState;
  }


  int counter = 0;
  @Override
  public void periodic() {
    if(!DriverStation.isTest()) {
      if(homing){
        //targetPosition = positionEncoder.getPosition() - Constants.intakeSubsystem.kHomingVel;
  
        if(intakeHomeLimit.get() == Constants.intakeSubsystem.kHomeLimitPressed){
          homing = false;
          positionPID.setReference(0, ControlType.kVelocity);
          positionMotor.set(0);   
          positionEncoder.setPosition(Constants.intakeSubsystem.kHomingPosition - Constants.intakeSubsystem.kHomingOffset);
          targetPosition = Constants.intakeSubsystem.kHomingPosition;
        }
      }
    
      if(safeMode) {
        double tempTarget = safeModeLimiter.calculate(targetPosition);
        positionPID.setReference(tempTarget, ControlType.kPosition);
      } else {
        double tempTarget = positionLimiter.calculate(targetPosition);
        positionPID.setReference(tempTarget, ControlType.kPosition);
      }
  
      if(counter > 50) {
        System.out.println("========>> Target Position: " + targetPosition);
        System.out.println("========>> Intake Speed: " + positionMotor.getOutputCurrent());
        counter = 0;
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
    } else {
      RobotContainer container = Robot.getRobotContainer();
      double joyX = container.assistantGetX();
      positionMotor.set(joyX / 5);
      
      if(counter > 50) {
        System.out.println("========>> Target Position: " + targetPosition);
        System.out.println("========>> Intake Speed: " + positionMotor.getOutputCurrent());
        System.out.println("=======>> Joystick Raw: " + joyX);
      }
    }
    counter++;
   }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
