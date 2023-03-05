package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
    private static LimeLight instance = null;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // constants//
    public final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 1.15; // how many degrees back is your limelight rotated from
                                                              // perfectly vertical
    public final double LIMELIGHT_LENSE_HEIGHT_CM = 97.5; // distance from the center of the Limelight lens to the floor
    public final double GOAL_HEIGHT_CM = 115; // distance from the target to the floor

    public final double distanceFromLimelightToGoalCentimetersPreset = 5; // distance to target in to calibrate the
    public final double LL_ROBOT_CENTER_DISTANCE = 18.75; // distance from LL to robot center in cm
    public final double LL_DISTANCE_FROM_ROBOT_EDGE = 16.2;
    // constants//

    public static LimeLight getInstance() {
        if (instance == null)
            instance = new LimeLight();
        
        return instance;
    }

    private LimeLight() {
    }

    public double getLimeLightXValue() {
        double x = table.getEntry("tx").getDouble(0.0);
        //SmartDashboard.putNumber("tx", x);
        return x;
    }

    public double getLimeLightYValue() {
        double y = table.getEntry("ty").getDouble(0.0);
        //SmartDashboard.putNumber("ty", y);
        return y;
    }

    public double getLimeLightAreaValue() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public double getTrueDistanceFromLL() {
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        // calculate distance

        double distanceFromLimelightToGoalCentimeters = ((GOAL_HEIGHT_CM - LIMELIGHT_LENSE_HEIGHT_CM)
                / Math.tan(angleToGoalRadians));
        //SmartDashboard.putNumber("Distance to goal from LL", distanceFromLimelightToGoalCentimeters);
        return distanceFromLimelightToGoalCentimeters;

    }

    /*
     * public double getLimeLightMountAngle() {
     * double LimeLightAngleDegrees = Math
     * .atan((GOAL_HEIGHT_CM - LIMELIGHT_LENSE_HEIGHT_CM) /
     * (distanceFromRobotToGoalCentimetersPreset + LL_DISTANCE_FROM_ROBOT_EDGE ))
     * + getLimeLightYValue();
     * SmartDashboard.putNumber("LimeLight mount angle", LimeLightAngleDegrees);
     * return LimeLightAngleDegrees;
     * }
     */

    /*
     * public double CrosshairX() {
     * NetworkTableEntry tx_Entry = table.getEntry("tx");
     * double tx = tx_Entry.getDouble(0.0);
     * //: constant: ALPHA = x;
     * double CrosshairX = 0 - (ALPHA - kx); // remember to change from -> // to
     * "  " because it's red
     * 
     * }
     */
        /* */
    public double FindAngleError() {
        NetworkTableEntry tx_Entry = table.getEntry("tx");
        double LL_ANGLE_FROM_TARGET = tx_Entry.getDouble(0.0);
        double LL_ANGLE_FROM_RCENTER = (90 - LL_ANGLE_FROM_TARGET) * (Math.PI / 180);
        // calc the B angle31
        double angleError = Math.atan((LL_ROBOT_CENTER_DISTANCE * Math.sin(LL_ANGLE_FROM_RCENTER))
                / (getTrueDistanceFromLL() - LL_ROBOT_CENTER_DISTANCE * Math.cos(LL_ANGLE_FROM_RCENTER)));
        angleError = (Math.PI / 2) - LL_ANGLE_FROM_RCENTER - angleError;
        angleError = angleError * (180 / Math.PI);
        //SmartDashboard.putNumber("Angle Error", angleError);
        return angleError;
    }
}