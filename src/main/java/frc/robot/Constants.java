// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;
// import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final String robotName = "Siren";
  public static final boolean kDebug = false;
  public static int kDefaultPipeline;


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

    public static final double kMaxRobotSpeed = 10.50;
    public static final double kDriveConversionFactor = 4.0;
    public static final double kAngleConversionFactor = 21.4285714286;
    public static final double kDriveGearRatio = 6.75;


    public static final PIDConstants TranslationPID = new PIDConstants(1.0, 0, 0.0);
    //3.5
    public static final PIDConstants RotationPID = new PIDConstants(0.4, 0, 0.0);
    public static final double MaxModuleSpeed = 4.5;

    // public static final double kPDriveHollonmic = 0.01;
    // public static final double kIDriveHollonmic = 0.0;
    // public static final double kDDriveHollonmic = 0.0;

    // public static final double kIAngleHollonmic = 0.1;
    // public static final double kDAngleHollonmic = 0.0;
    // public static final double kPAngleHollonmic = 0.0;
    public static final double kTeleopDeadzone = 0.1;
    public static final double kAngleSpeedMultiplier = 1;
    public static final double CommunityAutoTime = 2.25;
    public static final double kPositionOneYCF = 1.76;

    public static final boolean kEnableDynamicReplanning = true;
    public static final boolean kEnableIntialReplanning = true;

  }

  //TODO: need to set starter constant values

  public static class climberSubsystem {
   // public static final int kClimberMotorPort = 58;
    public static final int kClimberExtendRate = 0;
    public static final int kLeftClimberID = 25;
    public static final int kRightClimberID = 26;
    public static final int kLeftHomeLimitport = 2;
    public static final int kRightHomeLimitport = 1;
    public static final int kLockClimber = -1;
    public static final int kUnlockClimber = -1;
    public static final double kUpperPosition = 15.0;
    public static final double kHomingspeed = -0.5;
    public static final double kHomingPosition = 0.0;
    public static final boolean kHomePressed = false;
    public static final double kHomingOffset = 0;
    public static final boolean kRightEncoderInverted = false;
    public static final boolean kLeftEncoderInverted = false;
    public static final double kConversionFactor = 0.1;
    public static final double kLowerCautionPosition = 2.5;
    public static final double kCautionSpeedMultiplier = 0.25;
  }


  public static class shooterSubsystem {
    public static final boolean kTwoMotorUsed = true;

    public static final int kShootingMotorLPort = 27;
    public static final int kShootingmotorRPort = 28;
    public static final double kAmpshootingspeed = 385; //originally 500
    public static final double kstopshootingspeed = 0.0;
    public static final double kShootingspeed = 1800;
    // public static final double kShootingspeed = 3000; For only one working
    public static final double kShootingspeedlimit = 2000;
    // public static final double kShootingspeedlimit = 3500; 3500 for one working side

    public static final double kRightMotorP = 0.0004; //0.0002
    public static final double kRightMotorI = 0.0;
    public static final double kRightMotorD = 0.0;
	  public static final double kRightmotorFF = 0.00033;
    public static final double kRightMotorIZ = 0.0;

    public static final double kLeftMotorP = kRightMotorP;
    public static final double kLeftMotorI = kRightMotorI;
    public static final double kLeftMotorD = kRightMotorD;
    public static final double kLeftmotorFF = kRightmotorFF;
    public static final double kLeftMotorIZ = kRightMotorIZ;

    public static final double kStopShooterTime = 3.5; //try: 2.0

    // public static final double kStopShooterTime = 3.5;
    // public static final double kStartIntakeToShooter = 1.0;

    //public static final double kIntakeNoteInTime = 2.0;

    //public static final double kStopIntakeInTime = 0;



      //use these vals if we want to have same the PID for the shooter motors.
    // public static final double kShooterMotorP = 0.0;
    // public static final double kShooterMotorI = 0.0;
    // public static final double kShooterMotorD = 0.0;
	  // public static final double kShootermotorFF = 0.0;
    // public static final double kShooterMotorIZ = 0.0;
  }


 
  public static class intakeSubsystem {
    // public static final int kNoteMotorPort = 53;
    // public static final int kPositionMotorPort = 52;
    public static final double kSushiMotorPushOutSpeed = 0.75;
    public static final double kSushiMotorFeedShooterSpeed = 0.95;
    public static final double kStarMotorPushOutSpeed = 0.5;
    public static final double kStarMotorFeedShooterSpeed = 0.8;
    public static final double kStarMotorFeedAmpSpeed = 0.8;
    public static final double kSushiMotorFeedAmpSpeed = 0.95;
    public static final double kSushiMotorDrawInSpeed = 0.8;
    public static final double kStarMotorDrawInSpeed = 0.3;

    public static final double kStopNoteMotors = 0.0;
    public static final double kpositionUp = 14.4;
    public static final double kpositionDown = 174;
    public static final double kPositionAmp = 115;
    // public static final double kMaxIntakePosition = 170.0;
    // public static final double kMinIntakePosition = 5.0;
    // public static final double kPositionMotorupSpeed = 0.5;
    // public static final double kPositionMotorDownSpeed = 0.5;

    // public static final double kTargetPositionUp = 10.0; //Zero is the home position; this is inside the robot.  );
    // public static final double kTargetPositionDown = 170;

    public static final int kTopWheelMotorPortID = 23;
    public static final int kPositionMotorPortID = 24;
    public static final int kBottomStarMotorPortID = 22;

    public static final double kPositionRateLimit = 240.0;
    //public static final double kPositionInitialValue = 0.0;
    //public static final double kSafePositionRateLimit = 60.0;
    public static final boolean kHomeLimitPressed = false;
    public static final double kHomingVel = 0.1;
    public static final double kHomingPosition = 15;
    public static final double kHomingOffset = 0.0;
    public static final int kHomeLimitID = 0;
   
    public static final double kIntakeConversionFactor = 360.0; // Now using an absolute encoder //4.39007;

    public static final double kIntakeP = 0.01; //0.005
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0.0;
    public static final double kInitialIntakeFF = 0; //0.01; 

    public static final double kGravityFF = 0.0005; //0.0001
    public static final double kOffsetAngle = 0.0;
    public static final double kMaxFF = 0.001;
    public static final double kMinFF = -kMaxFF;
    public static final double kPositionEncoderOffset = 210.0; //approximately sets 0 to rear-facing horizontal //avoids meridian flip
    public static final double kPositionTolerance = 0.5; // in degrees
    public static final double kZeroOffsetBuffer = 10.0;
    public static final double kInjectNoteCorrectionStopTime = 0.6;
    public static final double kInjectNoteCorrectionStartTime = 0.25;


    public static final double kStopShooterTime = 3.5; // try 2.0
    public static final double kStartIntakeToShooter = 1.0;

    public static final double kIntakeNoteInTime = 1.0;
    public static final double kStopIntakeInTime = 3.5; // try: 2.0
    public static final double kStopIntakeAmpShootTime = 1.5;
    


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

    public static final int zeroGyroButton = 7;
    public static final int toggleFeildCentricButton = 8;
    public static final int toggleRobotCentricButton = 10;
    public static final int reverseDrive = 11;    
    public static final int driveForwardButton = 12;
    public static final int driveRightButton = 13;
    public static final int rotateCCWButton = 14;
    public static final int unReverseDrive = 16;
    // public static final int auto2driveForwardButton = 0;




  }
  public static class AssistantJoystick{
    public static final int joystick = 1;
    //public static final int shootButton = 1;
    public static final int shootLoadedNote = 1;    
    public static final int intakeExtendButton = 3;   
    public static final int intakeRetractButton = 4;   
    public static final int unlockClimberButtom = 5;
    public static final int intakeNoteInbutton = 7;
    public static final int intakeNoteOutbutton = 8;
    public static final int lockClimberButton = 10;
    public static final int extendClimberArmButton = 11;
    public static final int ampShootingButton = 13;
    public static final int retractClimberArmButton = 16;
    

  }

  public static final class UI {
    public static final double deadband = 0.05;
    public static final double delayCounter = 5.0;
}
}
