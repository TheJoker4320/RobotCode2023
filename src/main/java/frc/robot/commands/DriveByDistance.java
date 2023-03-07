// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Chassis;

public class DriveByDistance extends CommandBase {
  private final Chassis chassis;
  private final double rightSetpoint;
  private final double leftSetpoint;

  private final double timeout;
  private final Timer timer;

 public DriveByDistance(final Chassis chassis, double rightSetpoint, double leftSetpoint, double timeout) {
    this.chassis = chassis;
    this.rightSetpoint = rightSetpoint;
    this.leftSetpoint = leftSetpoint;

    this.timeout = timeout;
    timer = new Timer();

    addRequirements(chassis);
  }
  public DriveByDistance(final Chassis chassis, double rightSetpoint, double leftSetpoint) {
    this.chassis = chassis;
    this.rightSetpoint = rightSetpoint;
    this.leftSetpoint = leftSetpoint;

    this.timeout = Double.POSITIVE_INFINITY;
    timer = new Timer();

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.resetEncoders();
    timer.start();
    //SmartDashboard.putBoolean("We finished", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightValue = chassis.pidCalculate(chassis.getRightEncoder().getDistance(), rightSetpoint, 1);
    double leftValue = chassis.pidCalculate(chassis.getLeftEncoder().getDistance(), leftSetpoint, 1);

    chassis.setPowerToMotors(rightValue, leftValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stopDriving();
    //SmartDashboard.putBoolean("We finished", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return chassis.isAtSetpoint(1);
    boolean isAtRight = (chassis.getRightEncoder().getDistance() > rightSetpoint - PIDConstants.TOLERANCE_DRIVING) && 
                        (chassis.getRightEncoder().getDistance() < rightSetpoint + PIDConstants.TOLERANCE_DRIVING);

    boolean isAtLeft = (chassis.getLeftEncoder().getDistance() > leftSetpoint - PIDConstants.TOLERANCE_DRIVING) && 
                        (chassis.getLeftEncoder().getDistance() < leftSetpoint + PIDConstants.TOLERANCE_DRIVING);
    
    return (isAtRight && isAtLeft)
    || (timer.get() > timeout);
    
  }
}