package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // constants//
    // TODO: Correct the constats to the new robot.
    public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 1.5; // how many degrees back is your limelight rotated from
                                                             // perfectly vertical?
    public static final double LIMELIGHT_LENSE_HEIGHT_CM = 23.5; // distance from the center of the Limelight lens to the floor
    public static final double GOAL_HEIGHT_CM = 114.5; // distance from the target to the floor
    public static final double TRUE_LL_DISTANCE_FROM_RCENTER = 28.53507; //limelight distance from robot center directly
    public static final double HORIZONTAL_LL_DISTANCE_FROM_RCENTER = 19.1; //limelight distance from robot center horizontally
    public static final double VERTICAL_LL_DISTANCE_FROM_RCENTER = 21.2; //limelight distance from robot center vertically
    public static final double ANGLE_B_RADIAN = 0.837409; //angle opposite of the limelight near the base. i had not better name deal with it
    // constants//

    
     //First option
    /*
     public double getLimeLightXValue() {

     double[] x = table.getEntry("llpython").getDoubleArray(new double[4]);
     SmartDashboard.putNumber("tx", x[4]);
     return x[4];
     }
    */
    /*
     * Second option
     * public double getLimeLightHValue() {
     * double[] x = table.getEntry("llpython").getDoubleArray(new double[4]);
     * double h = x [4];
     * SmartDashboard.putNumber("tx", h);
     * return h;
     * }
     */
    
    public static double getLimeLightXValue(){
        double x = table.getEntry("tx").getDouble(0.0);
        SmartDashboard.putNumber("tx", x);
        return x;
    }

    public static double getLimeLightYValue() {
        double y = table.getEntry("ty").getDouble(0.0);
        SmartDashboard.putNumber("ty", y);
        return y;
    }
    public static double getLimeLightAreaValue() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public static double getTrueLLDistanceInCentimeters() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        NetworkTableEntry tx = table.getEntry("tx");
        double targetOffsetAngle_Horizontal = tx.getDouble(0.0);

        double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        // calculate distance
        double distanceFromLimelightToGoalCentimeters = (GOAL_HEIGHT_CM - LIMELIGHT_LENSE_HEIGHT_CM) / 
                ((180 / Math.PI) * Math.tan((Math.PI / 180) * angleToGoalDegrees) * 
                Math.cos((Math.PI / 180) * targetOffsetAngle_Horizontal));
        SmartDashboard.putNumber("Distance to goal from LL", distanceFromLimelightToGoalCentimeters);
        return distanceFromLimelightToGoalCentimeters;
    }

    public static double GetErrorDegreeFromTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry txEntry = table.getEntry("tx");
        double tx = txEntry.getDouble(0.0);

        double distanceFromLimelightToGoalCentimeters = getTrueLLDistanceInCentimeters();
        
        double degreeError = 90 - (180 - tx - (180 / Math.PI) * ANGLE_B_RADIAN - (180 / Math.PI) * Math.atan((
            (180 / Math.PI) * Math.sin((Math.PI / 180) * tx) * TRUE_LL_DISTANCE_FROM_RCENTER) /
            (distanceFromLimelightToGoalCentimeters - (180 / Math.PI) * Math.cos((Math.PI / 180) * tx) * TRUE_LL_DISTANCE_FROM_RCENTER)));
        SmartDashboard.putNumber("Degree To Go", degreeError);
        return degreeError;
    }
    //newly added command to be checked

}