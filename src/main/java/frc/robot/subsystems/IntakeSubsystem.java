// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;


public class IntakeSubsystem extends SubsystemBase {
 
enum IntakePositions{
Up,
Down
}


private IntakePositions intakePosition;
  private SendableCANSparkMax noteMotor;
  private SendableCANSparkMax positionMotor;
  private RelativeEncoder positionEncoder;
 
 
 
 
 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
 noteMotor = new SendableCANSparkMax(Constants.Intake.kNoteMotorPort, MotorType.kBrushless);
 positionMotor = new SendableCANSparkMax(Constants.Intake.kPositionMotorPort, MotorType.kBrushless);
 positionEncoder = positionMotor.getEncoder();


  }


  public void ejectNote(){
noteMotor.set(Constants.Intake.kEjectNoteSpeed);
  }
  public void injectNote(){
noteMotor.set(Constants.Intake.kInjectNoteSpeed);
  }
public void stopNote(){
  noteMotor.set(Constants.Intake.kStopNoteSpeed);

}
  public void positionUp(){
intakePosition = IntakePositions.Up;
  }
  public void positionDown(){
intakePosition = IntakePositions.Down;
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
    if(intakePosition == IntakePositions.Up){
      if(positionEncoder.getPosition() < 238){
        positionMotor.set(0.5);
      } else{
        positionMotor.set(0);
      }
    } else {
      if(positionEncoder.getPosition() > -10){
        positionMotor.set(-0.5);
      } else{
        positionMotor.set(0);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
