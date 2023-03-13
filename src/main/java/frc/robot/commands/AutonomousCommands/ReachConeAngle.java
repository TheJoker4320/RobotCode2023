// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ReachConeAngle extends CommandBase {
  private final Arm arm;
  /** Creates a new ReachConeAngle. */
  public ReachConeAngle(Arm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = arm.pidCalculate(arm.getDegrees(), ArmConstants.kANGLE_SETPOINT);

    if (output > ArmConstants.kMAX_ARM_AUTONOMOUS_SPEED)
      output = ArmConstants.kMAX_ARM_AUTONOMOUS_SPEED;
    else if (output < (-1 * ArmConstants.kMAX_ARM_AUTONOMOUS_SPEED))
      output = (-1 * ArmConstants.kMAX_ARM_AUTONOMOUS_SPEED);
    
    arm.Elevate(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    arm.setBrakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.pidAtSetpoint();
  }
}
