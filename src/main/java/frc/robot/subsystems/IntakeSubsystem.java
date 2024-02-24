// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.extensions.GravityAssistedFeedForward;
import frc.robot.extensions.SendableCANSparkMax;
import frc.robot.extensions.SparkMaxPIDTuner;


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
  private SparkAbsoluteEncoder absolutePositionEncoder;
  // private RelativeEncoder positionEncoder;
 
  private double targetPosition;
  private SparkPIDController positionPID;
  private SlewRateLimiter positionLimiter;
  private SlewRateLimiter safeModeLimiter;

  private DigitalInput intakeHomeLimit;
  private boolean homing;

  private boolean safeMode;

  private GravityAssistedFeedForward gravityFF;
 
  private SparkMaxPIDTuner positionPIDtuner;
 
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

    // positionEncoder = positionMotor.getEncoder();
    // positionEncoder.setPositionConversionFactor(Constants.intakeSubsystem.kIntakeConversionFactor);
    absolutePositionEncoder = positionMotor.getAbsoluteEncoder();
    absolutePositionEncoder.setPositionConversionFactor(360);
    absolutePositionEncoder.setZeroOffset(Constants.intakeSubsystem.kPositionEncoderOffset);
    targetPosition = Constants.intakeSubsystem.kpositionUp;
    positionPID = positionMotor.getPIDController();
    positionPID.setP(Constants.intakeSubsystem.kIntakeP);
    positionPID.setI(Constants.intakeSubsystem.kIntakeI);
    positionPID.setD(Constants.intakeSubsystem.kIntakeD);
    positionPID.setFF(Constants.intakeSubsystem.kIntakeFF);




    positionLimiter = new SlewRateLimiter(
      Constants.intakeSubsystem.kPositionRateLimit,
     -Constants.intakeSubsystem.kPositionRateLimit,
      Constants.intakeSubsystem.kPositionInitialValue);
    safeModeLimiter = new SlewRateLimiter(
      Constants.intakeSubsystem.kSafePositionRateLimit,
     -Constants.intakeSubsystem.kSafePositionRateLimit,
      Constants.intakeSubsystem.kSafePositionInitialValue);

    gravityFF = new GravityAssistedFeedForward(Constants.intakeSubsystem.kGravityFF, Constants.intakeSubsystem.kOffsetAngle);
    positionPIDtuner = new SparkMaxPIDTuner("PID Tuner", "Intake Position Motor", 1, positionPID);

    //Shuffleboard.getTab("LiveWindow").add(positionMotor);
    Shuffleboard.getTab("Intake").add("Position", positionMotor);
    Shuffleboard.getTab("Intake").add("Position Limitswitch", intakeHomeLimit);
    // Shuffleboard.getTab("Intake").add("Position Encoder", absolutePositionEncoder);
    Shuffleboard.getTab("Intake").addDouble("intake pos", this::getpos);
  }

  double getpos(){
    return absolutePositionEncoder.getPosition();
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
    targetPosition = Constants.intakeSubsystem.kpositionUp;
  }
  
  public void positionDown(){
    //intakePosition = IntakePositions.Down;
    targetPosition = Constants.intakeSubsystem.kpositionDown;
  }

  public void setHoming(boolean homingState){
    setSafeMode(true);
    homing = homingState;
    targetPosition = 5;
  }

  //TODO: need a command for exiting safe mode?
  public void setSafeMode(boolean safeModeState){
    safeMode = safeModeState;
  }

  public boolean isHome (){
    return intakeHomeLimit.get() == Constants.intakeSubsystem.kHomeLimitPressed;
  }




  int counter = 0;
  @Override
  public void periodic() {
    if(!DriverStation.isTest()) {
      if(intakeHomeLimit.get() == Constants.intakeSubsystem.kHomeLimitPressed){
        absolutePositionEncoder.setZeroOffset(-absolutePositionEncoder.getPosition() + Constants.intakeSubsystem.kZeroOffsetBuffer);
        if(homing){
          homing = false;
          targetPosition = Constants.intakeSubsystem.kpositionUp;
        } else {
          targetPosition = Math.max(Constants.intakeSubsystem.kHomingPosition, targetPosition);
        }
      }
      double FF = MathUtil.clamp(gravityFF.calculate(absolutePositionEncoder.getPosition()), Constants.intakeSubsystem.kMinFF, Constants.intakeSubsystem.kMaxFF);
      positionPID.setFF(FF);

      if(safeMode) {
        double tempTarget = safeModeLimiter.calculate(targetPosition);
        positionPID.setReference(tempTarget, ControlType.kPosition);
      } else {
        double tempTarget = positionLimiter.calculate(targetPosition);
        positionPID.setReference(tempTarget, ControlType.kPosition);
      }
  
      if(counter > 100) {
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
      
      if(counter > 100) {
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
