package frc.robot.LimelightLibrary;

import edu.wpi.first.networktables.NetworkTableInstance;



public class LimelightShortcuts {

    public double getTy() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    } 

    public double getTx() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getTv(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    } 

    public int getAprilTagID() {
        return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0);
    }
    
    public int getPipeline() {
       return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getInteger(0);
    }
    
}
