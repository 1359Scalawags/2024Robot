// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.extensions.SendableCANSparkMax;

public class ShooterSubsystem extends SubsystemBase {

  private SendableCANSparkMax shootingMotorL;
  private SendableCANSparkMax shootingMotorR;

  private double targetSpeed;
  private SparkPIDController speedPIDR;
  private SparkPIDController speedPIDL;

  enum ShooterSpeed {
    off,
    low,
    full

  }

  ShooterSpeed currentSpeed;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shootingMotorR = new SendableCANSparkMax(Constants.Shooter.kShootingmotorRPort, MotorType.kBrushless);
      shootingMotorR.restoreFactoryDefaults();
      shootingMotorR.setIdleMode(IdleMode.kCoast);
      shootingMotorR.setInverted(false);
      shootingMotorR.setSmartCurrentLimit(0);

    shootingMotorL = new SendableCANSparkMax(Constants.Shooter.kShootingMotorPort, MotorType.kBrushless);
      shootingMotorL.restoreFactoryDefaults();
      shootingMotorL.setIdleMode(IdleMode.kCoast);
      shootingMotorL.setInverted(false);
      shootingMotorL.setSmartCurrentLimit(0);

    currentSpeed = ShooterSpeed.off;
    speedPIDR = shootingMotorR.getPIDController();
      speedPIDR.setP(0);
      speedPIDR.setI(0);
      speedPIDR.setD(0);
      speedPIDR.setFF(0);
      speedPIDR.setIZone(0);
    speedPIDL = shootingMotorL.getPIDController();
      speedPIDL.setP(0);
      speedPIDL.setI(0);
      speedPIDL.setD(0);
      speedPIDL.setFF(0);
      speedPIDL.setIZone(0);
    


  }

  public void spinShootingMotor() {
    currentSpeed = ShooterSpeed.full;

    // shootingMotorL.set(Constants.Shooter.kShootingspeed);
    // shootingMotorR.set(-Constants.Shooter.kShootingspeed);
  }

  public void idleSpinShootingMotor() {
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
    // This method will be called once per scheduler run
    if (currentSpeed == ShooterSpeed.off) {
      targetSpeed = Constants.Shooter.kShootingspeed;

    } else if (currentSpeed == ShooterSpeed.low) {
      targetSpeed = Constants.Shooter.kShootingspeed;

    } else {
      targetSpeed = Constants.Shooter.kShootingspeed;

    }
    speedPIDR.setReference(targetSpeed, ControlType.kVelocity);
    speedPIDL.setReference(targetSpeed, ControlType.kVelocity);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
