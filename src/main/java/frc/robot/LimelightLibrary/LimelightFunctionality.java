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


        /**
         * @return Gets distance, in units given, from the robot on the xy-plane from the camera in given units. 
         * @param targetDistance    distance, in unit desinated from targetHeighth and cameraHeight, from the robot on the xy-plane from the camera, 
         * @param targetHeighth     Height, in units, from floor to target  
         * @param cameraHeight      height, in units, from floor to LL cameralens
         */
    public double getTargetDistanceFromCamera(double targetHeighth, double cameraHeight) {
        double zAxisHeight = Math.abs(targetHeighth - cameraHeight);
                // phi angle
        double azimuthAngle = LimelightShortcuts.getTx();
                // theta angle
        double polorAngle = LimelightShortcuts.getTy();
                //radial distance, this is also the distance from the camera to the target in the xyz-plane
        double radialDistance = zAxisHeight/Math.cos(polorAngle);
        
            //converting spherial cordnates to cartesian
        double yAxisWidth = radialDistance*Math.cos(polorAngle)*Math.cos(azimuthAngle); 

        double xAxisLength = radialDistance*Math.cos(polorAngle)*Math.sin(azimuthAngle);

            //Distance along the xy-plane
        double targetDistance = Math.sqrt(Math.pow(xAxisLength, 2)*Math.pow(yAxisWidth, 2));
        
        return targetDistance;
    }

    /**
     * <ul><li>If camera is above the center in relation to the target, xOffset is posative. 
     * If it is below, xOffset is negative.
     * <li>If camera is in front of the center in relation to the target, yOffset is posative
     * if it is  behind, yOffset is negative.</ul>
     * @param targetHeighth
     * @param cameraHeight 
     * @param xOffset the amount offset, up and down, from the center of robot to the camera
     * @param yOffset the amount offset, left and right, from the center of robot to the camera
     * @return Distance, in units given, from center of robot to the target
     */

    public double getTargetDistanceFromCenterOfRobot(double targetHeighth, double cameraHeight, double xOffset, double yOffset){
        double zAxisHeight = Math.abs(targetHeighth - cameraHeight);
                // phi angle
        double azimuthAngle = LimelightShortcuts.getTx();
                // theta angle
        double polorAngle = LimelightShortcuts.getTy();
                //radial distance, this is also the distance from the camera to the target in the xyz-plane
        double radialDistance = zAxisHeight/Math.cos(polorAngle);
        
            //converting spherial cordnates to cartesian
        double yAxisWidth = radialDistance*Math.cos(polorAngle)*Math.cos(azimuthAngle); 

        double xAxisLength = radialDistance*Math.cos(polorAngle)*Math.sin(azimuthAngle);

            //Applying offsets
        double xCenter = xAxisLength + xOffset;

        double yCenter = yAxisWidth + yOffset;


        double targetDistance = Math.sqrt(Math.pow(xCenter, 2)*Math.pow(yCenter, 2));

        return 1;
    }    


    // THIS DOES NOT GET DISTANCE FROM THE ROBOT
    //What happens if z = 0?
    // how does rotation of the robot affect the distance from the camera?

//     public double getAprilTagDistanceFromCamera(double tagHeighth, double cameraHeight) {
//         // this is based of the vector <i,j,k>, the magnatude of this vetor will give us the distance of the april tag to the camera.

//         // This is how you get 'k'
//         double zAxisHeight = Math.abs(tagHeighth - cameraHeight);

//         // this is how to get 'j'
//         double yAxisWidth = zAxisHeight/Math.tan(Math.abs(Math.toDegrees(LimelightShortcuts.getTy())));

//         // This is how you get 'i'
//         double xAxisLength = zAxisHeight/Math.tan(Math.abs(Math.toDegrees(LimelightShortcuts.getTx())));

// //The reason it changes when z=0 is because the distance between the two points is now acting as if it is in 2D space.
//         if (zAxisHeight == 0) {
//             double tagDistance = Math.sqrt((Math.pow(xAxisLength, 2)) + Math.pow(yAxisWidth, 2) + Math.pow(zAxisHeight, 2));
//             return tagDistance;
//         }
//         else {
//             double tagDistance = Math.sqrt((Math.pow(xAxisLength, 2)) + Math.pow(yAxisWidth, 2) + Math.pow(zAxisHeight, 2));
//             return tagDistance;
//         }
//     }
        // not sure if this works
    // public double getTargetDistance(double targetHeighth, double cameraHeight) {
    //     double zAxisHeight = Math.abs(targetHeighth - cameraHeight);
    //     double angleOfAzimuth = LimelightShortcuts.getTx();
    //     double angleOfElevation = LimelightShortcuts.getTy();

    //     double targetDistance = Math.sqrt(4);
    //     return targetDistance;
    // }

        // gets distance from cameras location on robot
}
