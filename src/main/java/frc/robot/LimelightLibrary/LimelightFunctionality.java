package frc.robot.LimelightLibrary;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightLibrary.*;

public class LimelightFunctionality {




  LimelightShortcuts LimelightShortcuts = new LimelightShortcuts();

    public void setDefaultPipeline (int DefaultPipelineNumber){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(DefaultPipelineNumber);
    }

    public void setPipeline (int pipelineNumber) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNumber);
    }

    // does this work for when the tag is offset to the x as well?
    // how can we assign heights for specific IDs 

    // public double getAprilTagDistance(double tagHeighth, double cameraHeight) {
    //     double ty = LimelightShortcuts.getTy();
    //     double elevation = Math.abs(tagHeighth - cameraHeight);
    //     double distance = (elevation)/Math.sin(ty);
    //     return distance;
    // }


    // THIS DOES NOT GET DISTANCE FROM THE ROBOT
    //What happens if z = 0?
    // how does rotation of the robot affect the distance from the camera?

    public double getAprilTagDistanceFromCamera(double tagHeighth, double cameraHeight) {
        // this is based of the vector <i,j,k>, the magnatude of this vetor will give us the distance of the april tag to the camera.

        // This is how you get 'k'
        double zAxisHeight = Math.abs(tagHeighth - cameraHeight);

        // this is how to get 'j'
        double yAxisWidth = zAxisHeight/Math.tan(Math.abs(Math.toDegrees(LimelightShortcuts.getTy())));

        // This is how you get 'i'
        double xAxisLength = zAxisHeight/Math.tan(Math.abs(Math.toDegrees(LimelightShortcuts.getTx())));

//The reason it changes when z=0 is because the distance between the two points is now acting as if it is in 2D space.
        if (zAxisHeight == 0) {
            double tagDistance = Math.sqrt((Math.pow(xAxisLength, 2)) + Math.pow(yAxisWidth, 2) + Math.pow(zAxisHeight, 2));
            return tagDistance;
        }
        else {
            double tagDistance = Math.sqrt((Math.pow(xAxisLength, 2)) + Math.pow(yAxisWidth, 2) + Math.pow(zAxisHeight, 2));
            return tagDistance;
        }
    }

    public double getTargetDistance(double targetHeighth, double cameraHeight) {
        double zAxisHeight = Math.abs(targetHeighth - cameraHeight);
        double angleOfAzimuth = LimelightShortcuts.getTx();
        double angleOfElevation = LimelightShortcuts.getTy();

        double targetDistance = Math.sqrt(4);
        return targetDistance;
    }

}
