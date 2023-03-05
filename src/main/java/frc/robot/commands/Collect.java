// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

/** An example command that uses an example subsystem. */
public class Collect extends CommandBase{
  private final Collector collectorSubSystem;

  
  public Collect(Collector collectorSubSystem) {
    this.collectorSubSystem = collectorSubSystem;

    addRequirements(collectorSubSystem); // Added to avoid errors when more than one subsystem is using the command.
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    collectorSubSystem.collect();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectorSubSystem.stopCollecting();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}