// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;

public class Collector extends SubsystemBase {

  private final WPI_TalonSRX Front_Motor;
  private static Collector single_instance = null;

  private Collector() {
    Front_Motor = new WPI_TalonSRX(CollectorConstants.FRONT_MOTOR_PORT);
    Front_Motor.configFactoryDefault();
    Front_Motor.setInverted(true);
  }

  public static Collector getInstance() {
    if (single_instance == null)
      single_instance = new Collector();

    return single_instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void collect() {
    Front_Motor.set(CollectorConstants.COLLECTING_SPEED);
  }

  public void eject() {
    Front_Motor.set(-CollectorConstants.COLLECTING_SPEED);
  }

  public void stopCollecting() {
    Front_Motor.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}