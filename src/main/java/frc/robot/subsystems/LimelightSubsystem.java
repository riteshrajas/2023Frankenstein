package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    // private Variable declarartion
    private NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightVariables.moduleName);

    public LimelightSubsystem() {}

    public static class LimelightVariables {
        public static String moduleName = "limelight";// for furure refernce
        public static double pipelineName;// for furure refernce
        public static double ledMode;// for furure refernce
        public static double tx;// target X - position
        public static double ty;// target Y = position
        public static double ta;// target area
        public static boolean tv;// Weather or not the limelight sees a target
        public static double tid;// Target id
    }

    public void periodic_variables_read(){
        LimelightVariables.tx = table.getEntry("tx").getDouble(0.0);
        LimelightVariables.ty = table.getEntry("ty").getDouble(0.0);
        LimelightVariables.ta = table.getEntry("ta").getDouble(0.0);
        LimelightVariables.tv = table.getEntry("tv").getDouble(0.0) == 1.0;
        LimelightVariables.ledMode = table.getEntry("ledMode").getDouble(1.0);
        LimelightVariables.pipelineName = table.getEntry("pipeline").getDouble(0);
        LimelightVariables.tid = table.getEntry("tid").getDouble(0.0);
    }
    public void periodic_variables_write(){
        // SmartDashboard.putBooleanArray("Target", LimelightFunctions.getTx());
        SmartDashboard.putNumber("LimelightX", LimelightFunctions.getTx());
        SmartDashboard.putNumber("LimelightY", LimelightFunctions.getTy());
        SmartDashboard.putNumber("LimelightArea", LimelightFunctions.getTa());
        SmartDashboard.putNumber("LimelightPipeline", LimelightVariables.pipelineName);
        SmartDashboard.putNumber("LimelightLEDMode", LimelightVariables.ledMode);
        SmartDashboard.putNumber("LimelightTargetID", LimelightVariables.tid);
    }

    @Override
    public void periodic() {
        periodic_variables_read();
        periodic_variables_write();
    
    };

    public static class LimelightFunctions {
        public static double getTx() {
            return LimelightVariables.tx;
        }

        public static double getTy() {
            return LimelightVariables.ty;
        }

        public static double getTa() {
            return LimelightVariables.ta;
        }

        public static boolean getTv() {
            return LimelightVariables.tv;
        }
        /**
         * @return Returns the ID of the Tag
         */
        public static double getTid() {
            return LimelightVariables.tid;
        }
    }

    /**
    * Determines the direction of the limelight based on the position of the camera.
    *
    * @param threshold This threshold is used to control the frequency of direction changes.
    *                 (A higher threshold leads to slower changes in direction.)
    * @return A String indicating the direction of the limelight. Possible values are "left," "right," or "centre".
    */
    public String getLateral(double threshold) {
        if (LimelightVariables.tx < threshold) {return "left";} else if (LimelightVariables.tx < -threshold) {return "right";} else {return "centre";}
    }

    /**
     * Determines the robot movement direction based on the change in the limelight's target area.
     *
     * @param initialArea The initial area of the target when the movement command is started.
     * @param currentArea The current area of the target.
     * @param areaChangeThreshold The threshold for considering a significant change in area.
     * @return A string indicating the movement direction: "forward", "backward", or "stop".
     */
    public String getTraverse(double initialArea, double currentArea, double areaChangeThreshold) {
        double areaChange = currentArea - initialArea;
        if (areaChange > areaChangeThreshold) {return "forward";} else if (areaChange < -areaChangeThreshold) {return "backward";} else {return "stop";}
}

    /**
     * Calculates the distance to a target using Limelight camera data.
     *
     * @param h1 The height of the camera above the floor.
     * @param h2 The height of the target.
     * @param a1 The mounting angle of the camera in degrees.
     * @param a2 The Y angle to the target in degrees.
     * @return The estimated distance to the target in inches.
     */
     public static class LimelightDistanceCalculator {

        // Function to convert degrees to radians
        private static double toRadians(double degrees) {
            return degrees * (Math.PI / 180.0);
        }
    
        // Standalone function to calculate distance using Limelight data
        public static double calculateDistance(double h1, double h2, double a1, double a2) {
            // Convert angles to radians
            double a1Radians = toRadians(a1);
            double a2Radians = toRadians(a2);
    
            // Calculate distance using the formula
            double distance = (h2 - h1) / Math.tan(a1Radians + a2Radians);
    
            return distance;
        }
     }

    /**
     * @param pipeline 0-9
     */
    public void setPipeline(Integer pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
   
    /**
     * @param mode 0 = Off, 1 = Blink, 2 = On
     */
    public void setLEDMode(Integer mode) {
        table.getEntry("ledMode").setNumber(mode);
    }
    
    /**
     * @param mode 0 = Vision Processing, 1 = Driver Cam 
     */
    public void setCameraMode(Integer mode) {
        table.getEntry("camMode").setNumber(mode);
    }

    /**
     * @return returns true if the limelight sees a target, false otherwise.
     */
    public boolean seesTarget() {
        return LimelightFunctions.getTv();
    }
    
    public Translation2d getDirection(double d) {
        return null;
    }

    public double getTag_id(){
        return (double) LimelightVariables.tid;
    }

    
}


// Documents used to Code this file
// https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory
// https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
// https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance#:~:text=%22d%20%3D%20(h2%2Dh1)%20/%20tan(a1%2Ba2)%22
