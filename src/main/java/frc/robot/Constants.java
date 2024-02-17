// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

/*
 * 
 * NOT FINAL VALUES FOR SWERE
 * 
 */
  public static class swerveSubsystem {
    //public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    //public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    //public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

    public static final double kMaxRobotSpeed = 7.5;
    public static final double kDriveConversionFactor = 4.0;
    public static final double kAngleConversionFactor = 12.8;
    public static final double kDriveGearRatio = 6.75;


    public static final PIDConstants TranslationPID = new PIDConstants(1, 0, 0.1);
    public static final PIDConstants RotationPID = new PIDConstants(3.5, 0, 0.1);
    public static final double MaxModuleSpeed = 4.5;

    // public static final double kPDriveHollonmic = 0.01;
    // public static final double kIDriveHollonmic = 0.0;
    // public static final double kDDriveHollonmic = 0.0;

    // public static final double kIAngleHollonmic = 0.1;
    // public static final double kDAngleHollonmic = 0.0;
    // public static final double kPAngleHollonmic = 0.0;
  }

  public static class ClimberSubsystem {
   // public static final int kClimberMotorPort = 58;
    public static final int kClimberUperlimitswitchport = 1;
    public static final int kClimberExtendRate = 0;
    public static final int kClimberLowerlimitswitchport = 0;
  }


  public static class Shooter {
   // public static final int kShootingMotorPort = 55;
    public static final double kShootingspeed = 0.5;
//public static final int kShootingmotorPort = 54;
    public static final int kIdleshootingspeed = 0;
    public static final int kstopshootingspeed = 0;
  }


 
  public static class Intake {
    // public static final int kNoteMotorPort = 53;
    // public static final int kPositionMotorPort = 52;
    public static final double kEjectNoteSpeed = 0.5;
    public static final double kInjectNoteSpeed = -0.5;
    public static final int kStopNoteSpeed = 0;
    public static final double kpositionUp = 0;
    public static final double kpositionDown = 0;

    public static final double kMaxIntakePosition = 238.0;
    public static final double kMinIntakePosition = -10.0;
    public static final double kPositionMotorupSpeed = 0.5;
    public static final double kPositionMotorDownSpeed = 0.5;





  }
 
  
 
  public static class visionSubsystem {
    public static final int PDHCANID = 1;
    public static final int CAM_WIDTH = 320;
    public static final int CAM_HEIGHT = 240;
    public static final int CAM_FPS = 15;

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0; 
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 0; 
    // distance from the target to the floor
    double goalHeightInches = 0; 
  }

  public static class ExampleSubsystem {
    public static final int exampleMotorID = 33;
  }
  public static class DriverJoystick{
    public static final int joystick = 0;
    public static final int shootButton = 1;
    public static final int extendArmButton = 2;
    public static final int retractArmButton = 3;
    public static final int intakeBeltButton = 4;
    public static final int intakeExtendButton = 5;
    public static final int intakeWheelsOnbutton = 6;
    public static final int zeroGyroButton = 7;
    public static final int toggleFeildCentricButton = 8;


  }
  public static class AssistantJoystick{
    public static final int joystick = 1;


  }

  public static final class UI {
    public static final double deadband = 0.05;
    public static final double delayCounter = 5.0;
}


}
