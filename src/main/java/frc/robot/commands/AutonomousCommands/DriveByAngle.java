// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Chassis;

public class DriveByAngle extends CommandBase {
  private final Chassis chassis;
  private double startingAngle;

  private double angleSetpoint;
  private boolean reachedAngle;
  /** Creates a new DriveByAngle. */
  public DriveByAngle(Chassis chassis, double angleSetpoint) {
    this.chassis = chassis;
    this.angleSetpoint = angleSetpoint;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingAngle = chassis.getStartAngle();
    reachedAngle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (chassis.getAngle() > (startingAngle + angleSetpoint) - PIDConstants.kDBA_TOLERANCE && chassis.getAngle() < (startingAngle + angleSetpoint) + PIDConstants.kDBA_TOLERANCE)
      reachedAngle = true;
    else
      chassis.setPowerToMotors(PIDConstants.kSPEED_ON_RAMP, PIDConstants.kSPEED_ON_RAMP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPowerToMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reachedAngle;
  }
}
