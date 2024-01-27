// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private CANSparkMax exampleMotor;
  
  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem() {
    exampleMotor = new CANSparkMax(Constants.ExampleSubsystem.exampleMotorID, MotorType.kBrushless);


    
  }

  public final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

public Rotation2d getGyroscopeRotation() {
      return m_navx.getRotation2d();
  
      // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
      // return Rotation2d.fromDegrees(360.0 - navx.getYaw());
    }

  //this is awsome

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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
