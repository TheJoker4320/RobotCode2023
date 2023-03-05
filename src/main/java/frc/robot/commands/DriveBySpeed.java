// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveBySpeed extends CommandBase {
  private final Chassis chassis;
  private final DoubleSupplier forwardSpeed;
  private final DoubleSupplier rotatingSpeed;

  public DriveBySpeed(final Chassis chassis, final DoubleSupplier forwardSpeed, final DoubleSupplier rotatingSpeed) {
    this.chassis = chassis;
    this.forwardSpeed = forwardSpeed;
    this.rotatingSpeed = rotatingSpeed;

    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.drive(-1 * forwardSpeed.getAsDouble(), -1 * rotatingSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}