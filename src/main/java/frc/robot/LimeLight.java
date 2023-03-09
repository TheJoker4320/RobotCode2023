package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // constants//
    // TODO: Correct the constats to the new robot.
    public static final double LL_DEGREE = 2.15; //1.5 how many degrees back is your limelight rotated from
                                                             // perfectly vertical?
    public static final double LL_LENSE_HEIGHT_CM = 98; // distance from the center of the Limelight lens to the floor
    public static final double GOAL_HEIGHT_CM = 117.5; // distance from the target to the floor
    //public static final double GOAL_HEIGHT_CM = 114.5; // distance from the target to the floor
    public static final double LL_DISTANCE_FROM_CENTER = 23.4917; //limelight distance from robot center directly ((13.1^2 + 19.5^2)^0.5)
    public static final double HORIZONTAL_LL_DISTANCE_FROM_RCENTER = 13.1; //limelight distance from robot center horizontally
    public static final double VERTICAL_LL_DISTANCE_FROM_RCENTER = 19.5; //limelight distance from robot center vertically
    public static final double LL_AGGLE_FROM_CENTER_RADS = Math.tan(13.1/19.5) ; // angle opposite of the limelight near the base. i had not better name deal with it
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

        NetworkTableEntry ty_entry = table.getEntry("ty");
        double ty = ty_entry.getDouble(0.0);

        NetworkTableEntry tx_entry = table.getEntry("tx");
        double tx = tx_entry.getDouble(0.0);
        double txRads = Math.toRadians(tx);

        double angleToGoalDegrees = LL_DEGREE + ty;
        double angleToGoalRadians =  Math.toRadians(angleToGoalDegrees);
        // calculate distance
        double distanceFromGoal = (GOAL_HEIGHT_CM - LL_LENSE_HEIGHT_CM) / 
                (Math.tan(angleToGoalRadians) * Math.cos(txRads));
        SmartDashboard.putNumber("Distance to goal from LL", distanceFromGoal);
        return distanceFromGoal;
    }

    public static double GetErrorDegreeFromTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableEntry tx_entry = table.getEntry("tx");
        double tx = tx_entry.getDouble(0.0);
        double txRads = Math.toRadians(tx);

        double distanceFromLimelightToGoalCentimeters = getTrueLLDistanceInCentimeters();

/*        double degreeError = 90 - tx - (180 / Math.PI) * LL_AGGLE_FROM_CENTER_RADS - (180 / Math.PI) * Math.atan((
            (180 / Math.PI) * Math.sin((Math.PI / 180) * tx) * LL_DISTANCE_FROM_CENTER) /
            (distanceFromLimelightToGoalCentimeters - (180 / Math.PI) * Math.cos((Math.PI / 180) * tx) * LL_DISTANCE_FROM_CENTER)));*/

        /*double degreeError = 90 - tx - Math.toDegrees(Math.atan(
        (distanceFromLimelightToGoalCentimeters -Math.sin(txRads) * LL_DISTANCE_FROM_CENTER) /
        (Math.cos(txRads) * LL_DISTANCE_FROM_CENTER)));*/

        double degreeError = - 90 + tx + Math.toDegrees(Math.atan(
        (distanceFromLimelightToGoalCentimeters -Math.sin(txRads) * LL_DISTANCE_FROM_CENTER) /
        (Math.cos(txRads) * LL_DISTANCE_FROM_CENTER)));

        SmartDashboard.putNumber("Degree To Goal", degreeError);
        return degreeError;
    }
    //newly added command to be checked

}