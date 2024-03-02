// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.extensions.SendableCANSparkMax;

public class ShooterSubsystem extends SubsystemBase {

  private SendableCANSparkMax shootingMotorL;
  private SendableCANSparkMax shootingMotorR;

  private double targetSpeed;
  private SparkPIDController speedPIDR;
  private SparkPIDController speedPIDL;

  private SlewRateLimiter shooterLimiter;

  enum ShooterSpeed {
    off,
    low,
    full

  }

  ShooterSpeed currentSpeed;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shootingMotorR = new SendableCANSparkMax(Constants.shooterSubsystem.kShootingmotorRPort, MotorType.kBrushless);
      shootingMotorR.restoreFactoryDefaults();
      shootingMotorR.setIdleMode(IdleMode.kCoast);
      shootingMotorR.setInverted(true);

    shootingMotorL = new SendableCANSparkMax(Constants.shooterSubsystem.kShootingMotorLPort, MotorType.kBrushless);
      shootingMotorL.restoreFactoryDefaults();
      shootingMotorL.setIdleMode(IdleMode.kCoast);
      shootingMotorL.setInverted(false);

    currentSpeed = ShooterSpeed.off;


    speedPIDR = shootingMotorR.getPIDController();
      speedPIDR.setP(Constants.shooterSubsystem.kRightMotorP);
      speedPIDR.setI(Constants.shooterSubsystem.kRightMotorI);
      speedPIDR.setD(Constants.shooterSubsystem.kRightMotorD);
      speedPIDR.setFF(Constants.shooterSubsystem.kRightmotorFF);
      speedPIDR.setIZone(Constants.shooterSubsystem.kRightMotorIZ);
    speedPIDL = shootingMotorL.getPIDController();
      speedPIDL.setP(Constants.shooterSubsystem.kLeftMotorP);
      speedPIDL.setI(Constants.shooterSubsystem.kLeftMotorI);
      speedPIDL.setD(Constants.shooterSubsystem.kLeftMotorD);
      speedPIDL.setFF(Constants.shooterSubsystem.kLeftmotorFF);
      speedPIDL.setIZone(Constants.shooterSubsystem.kLeftMotorIZ);
    
    shooterLimiter = new SlewRateLimiter
      (Constants.shooterSubsystem.kTwoMotorUsed ? Constants.shooterSubsystem.kShootingspeedlimit : 3500);
  }

  public void spinShootingMotor() {
    currentSpeed = ShooterSpeed.full;

    // shootingMotorL.set(Constants.Shooter.kShootingspeed);
    // shootingMotorR.set(-Constants.Shooter.kShootingspeed);
  }

  public void ampSpinShootingMotor() {
    currentSpeed = ShooterSpeed.low;

    // shootingMotorL.set(Constants.Shooter.kIdleshootingspeed);
    // shootingMotorR.set(-Constants.Shooter.kIdleshootingspeed);

  }

  public void stopSpinShootingMotor() {
    currentSpeed = ShooterSpeed.off;

    // shootingMotorL.set(Constants.Shooter.kstopshootingspeed);
    // shootingMotorR.set(-Constants.Shooter.kstopshootingspeed);
  }

  @Override
  public void periodic() {
    if(!DriverStation.isTest()){
      // This method will be called once per scheduler run
      if (currentSpeed == ShooterSpeed.off) {
        targetSpeed = Constants.shooterSubsystem.kstopshootingspeed;
      }
      else if (currentSpeed == ShooterSpeed.low) {
        targetSpeed = Constants.shooterSubsystem.kAmpshootingspeed;
      }
      else {
        targetSpeed = Constants.shooterSubsystem.kShootingspeed;
      }
      double limitSpeed = shooterLimiter.calculate(targetSpeed);
      //speedPIDR.setReference(0, ControlType.kVelocity); //Right motor
      speedPIDR.setReference(Constants.shooterSubsystem.kTwoMotorUsed ? limitSpeed : 0, ControlType.kVelocity); //Right motor
      speedPIDL.setReference(Constants.shooterSubsystem.kTwoMotorUsed ? limitSpeed : 3000, ControlType.kVelocity); // Left motor
    }
    else {
      double joyX = Robot.getRobotContainer().driverGetX()/2;
      shootingMotorL.set(joyX);
      shootingMotorR.set(joyX);



    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
