// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class StabalzieOnRamp extends CommandBase {

  private final double ANGLE_TOLORENCE = 1.5;
  private final double preANGLE_TOLERANCE = 5;

  private final Chassis chassis;
  private double startingAngle;
  private boolean stabalizing;
  private boolean stabalized;
  private boolean justStabalized;
  private Timer timer;

  public StabalzieOnRamp(Chassis chassis) {
    this.chassis = chassis;
    stabalizing = false;
    stabalized = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    startingAngle = chassis.getStartAngle();
    justStabalized = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!stabalizing) {
      double drivingSpeed = chassis.pidCalculate(chassis.getAngle(), startingAngle, 2);
      double speed = drivingSpeed < 0 ? drivingSpeed * 1.2 : drivingSpeed;
      chassis.setPowerToMotors(speed, speed);
    }
    else {
      chassis.lock();
    }

    if (chassis.getAngle() < startingAngle + preANGLE_TOLERANCE && chassis.getAngle() > startingAngle - preANGLE_TOLERANCE) {
      if (!justStabalized) {
        justStabalized = true;
        //Start timer
        timer.start();
      }
      else {
        justStabalized = false;
      }
      stabalizing = true;
      chassis.setMotorsBrake();

      if (chassis.getAngle() < startingAngle + ANGLE_TOLORENCE && chassis.getAngle() > startingAngle - ANGLE_TOLORENCE) {
        stabalized = true;
      }
    } 
    else {
      timer.stop();
      timer.reset();
      stabalizing = false;
      stabalized = false;
      justStabalized = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.drive(0, 0);
    chassis.setMotorsBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stabalized && (timer.get() > 2);
  }
}