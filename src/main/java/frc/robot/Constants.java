// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class ClimberSubsystem {
    public static final int kClimberMotorPort = 0;
  }


  public static class Shooter {
    public static final int kShootingMotorPort = 7;
  }


 
  public static class suckingMotor {
    public static final int kSuckingMotorPort = 1;
  }
 
  
 
  public static class visionSubsystem {
    public static final int PDHCANID = 1;
    public static final int CAM_WIDTH = 320;
    public static final int CAM_HEIGHT = 240;
    public static final int CAM_FPS = 15;
  }

  public static class ExampleSubsystem {
    public static final int exampleMotorID = 33;
  }

}
