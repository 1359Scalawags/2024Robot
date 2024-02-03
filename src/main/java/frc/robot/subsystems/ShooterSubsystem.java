// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;

public class ShooterSubsystem extends SubsystemBase {

private SendableCANSparkMax shootingMotorL;
private SendableCANSparkMax shootingMotorR;



  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    // shootingMotorR = new SendableCANSparkMax(Constants.Shooter.kShootingmotorRPort, MotorType.kBrushless);
    // shootingMotorL = new SendableCANSparkMax(Constants.Shooter.kShootingMotorPort, MotorType.kBrushless);



  }
 public void spinShootingMotor(){
  shootingMotorL.set(Constants.Shooter.kShootingspeed);
  shootingMotorR.set(-Constants.Shooter.kShootingspeed);
 }


 public void idleSpinShootingMotor(){
  shootingMotorL.set(Constants.Shooter.kIdleshootingspeed);
  shootingMotorR.set(-Constants.Shooter.kIdleshootingspeed);

 }
 public void stopSpinShootingMotor(){
  shootingMotorL.set(Constants.Shooter.kstopshootingspeed);
  shootingMotorR.set(-Constants.Shooter.kstopshootingspeed);
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
