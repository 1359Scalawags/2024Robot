package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.LimelightLibrary.LimelightShortcuts;
import frc.robot.Constants.swerveSubsystem;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//positive x value, right negative, left
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {

    LimelightShortcuts LimelightShortcuts = new LimelightShortcuts();

    public enum LimelightModes {
        vision,
        driver
    }

    public enum USBCameras {
        TopCamera,
        BottomCamera
    }
    

   
    double x, y, area;
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = getLimelightEntry("tx");
    NetworkTableEntry ty = getLimelightEntry("ty");
    NetworkTableEntry ta = getLimelightEntry("ta");
    NetworkTableEntry tv = getLimelightEntry("tv");
    NetworkTableEntry ledMode = getLimelightEntry("ledMode");
    NetworkTableEntry camMode = getLimelightEntry("camMode");
    NetworkTableEntry pipeline = getLimelightEntry("pipeline");
    NetworkTableEntry botPoseEntry = table.getEntry("botpose");
    HttpCamera camera = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpeg", HttpCameraKind.kMJPGStreamer);
    private double[] botPose;


    public VisionSubsystem() {

        CameraServer.startAutomaticCapture(camera);
        botPose = botPoseEntry.getDoubleArray(new double[6]);

        //AprilTagFieldLayout layout;
        //april tag fmap intialization
       // //TODO: change fmap to 2024 feild, kavi has the file (add to constants).
        //this.aprilTagFieldLayout = layout;
    //    AprilTagFieldLayout aprilTagFieldLayout;
    //     try {
    //   aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      
    //     Optional<Alliance> alliance = DriverStation.getAlliance();
    //   aprilTagFieldLayout.setOrigin(alliance.equals(Alliance.Blue) ?
    //       OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    // } 
    //     catch(IOException e) {
    //   DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    //   layout = null;
    // }

        //TODO: make functions in SwerveSubsystem to be used here
    // poseEstimator =  new SwerveDrivePoseEstimator(
    //     SwereSubsystemConstants.KINEMATICS,
    //     SwereSubsystem.getGyroscopeRotation(),
    //     SwereSubsystem.getModulePositions(),
    //     new Pose2d(),
    //     stateStdDevs,
    //     visionMeasurementStdDevs);

    // tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    // tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

    


        // limelight initialization
        setCamMode(LimelightModes.vision);

    }

    public static void setCamMode(LimelightModes mode) {
        getLimelightEntry("camMode").setNumber(mode.ordinal());
    }

  public void setPipeline(int pipelineNumber){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNumber);
    }

    public void setDefaultPipeline(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(Constants.kDefaultPipeline);
    }

    int counter = 0;
    int timer = 0;

//*********************************************************************************
        // Functions for LL library.
        
    public void testTy() {
        double ty = LimelightShortcuts.getTy();
        System.out.println("------------------getTy() output---------------------------" + ty);
    }
    
    public void testTx() {
        double tx = LimelightShortcuts.getTx();
        System.out.println("------------------getTx() output---------------------------" + tx);
    }

    public void testTv() {
        double tv = LimelightShortcuts.getTv();
        System.out.println("------------------getTv() output---------------------------" + tv);
    }

    public void getPipe() {
        double pipeline = LimelightShortcuts.getPipeline();
        System.out.println("------------------getPipeline() output-----------------------" + pipeline);
    }

    public void testTagID() {
        int tid = LimelightShortcuts.getAprilTagID();
        System.out.println("-------------------AprilTag ID output---------------------------" + (int) tid);
    }

    public void getBasicValues() {
        double ty = LimelightShortcuts.getTy();
        double tx = LimelightShortcuts.getTx();
        double tv = LimelightShortcuts.getTv();
        double pipeline = LimelightShortcuts.getPipeline();
        int tid = LimelightShortcuts.getAprilTagID();

        System.out.println("------------------getTy() output---------------------------" + ty);
        System.out.println("------------------getTx() output---------------------------" + tx);
        System.out.println("------------------getTv() output---------------------------" + tv);
        System.out.println("------------------getPipeline() output-----------------------" + pipeline);
        System.out.println("-------------------AprilTag ID output---------------------------" + (int) tid);
}

//**************************************************************************************

    @Override
    public void periodic() {
        // read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double[] pose = botPoseEntry.getDoubleArray(new double[6]);


        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumberArray("Limelight", pose);
    

        //TODO: we are puting the same numbers to the dashboard on the delay and before it.

                // post to smart dashboard periodically
        if (counter > Constants.UI.delayCounter) {
            SmartDashboard.putNumber("LimelightX", x);
            SmartDashboard.putNumber("LimelightY", y);
            SmartDashboard.putNumber("LimelightArea", area);
            SmartDashboard.putNumberArray("Limelight", pose);
            counter = 0;
        }
        counter++;

        //*******************************************************
        double ty = LimelightShortcuts.getTy();
        double tx = LimelightShortcuts.getTx();
        double tv = LimelightShortcuts.getTv();
        int pipeline = LimelightShortcuts.getPipeline();
        int tid = LimelightShortcuts.getAprilTagID();

        if (timer >= 5) {
            System.out.println("------------------getTy() output---------------------------" + ty);
            System.out.println("------------------getTx() output---------------------------" + tx);
            System.out.println("------------------getTv() output---------------------------" + tv);
            System.out.println("------------------getPipeline() output---------------------" + (int) pipeline);
            System.out.println("------------------AprilTag ID output-----------------------" + (int) tid);

            timer = 0;
        }
        timer++;
        // loop by Gavan and kavi :)


        //********************************************************      
    }

    // poseEstimator.update(
    //     SwereSubsystemSubsystem.getGyroscopeRotation(),
    //     SwereSubsystemSubsystem.getModulePositions());
      
    //     field2d.setRobotPose(getCurrentPose());
        
      

    // }

    private static NetworkTableEntry getLimelightEntry(String key) {
        if (table == null) {
            table = NetworkTableInstance.getDefault().getTable("limelight");
        }
        return table.getEntry(key);
    }

    @Override
    public void simulationPeriodic() {

    }

    public Double getTargetX() {
        return tx.getDouble(0.0);
    }

    public Double getTargetY() {
        return ty.getDouble(0.0);
    }

    public Double getTargetArea() {
        return ta.getDouble(0.0);
    }
}