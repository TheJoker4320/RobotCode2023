// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private DigitalInput microSwitch;
  DutyCycleEncoder encoder;

  private final WPI_VictorSPX Master_Motor;
  private final WPI_VictorSPX Slave_Motor;
  private final MotorControllerGroup motorControllerGroup;

  private final PIDController armPidController;

  private static Arm single_instance = null;

  private Arm() {
    microSwitch = new DigitalInput(ArmConstants.MICRO_SWITCH_PORT);
    encoder = new DutyCycleEncoder(ArmConstants.ENCODER_CHANNEL);
    encoder.setDistancePerRotation(360);

    Master_Motor = new WPI_VictorSPX(ArmConstants.MASTER_PORT);
    Master_Motor.configFactoryDefault();

    Slave_Motor = new WPI_VictorSPX(ArmConstants.SLAVE_PORT);
    Slave_Motor.configFactoryDefault();

    setBrakeMotor();
    motorControllerGroup = new MotorControllerGroup(Master_Motor, Slave_Motor);

    armPidController = new PIDController(ArmConstants.kP_ARM, ArmConstants.kI_ARM, ArmConstants.kD_ARM);
    armPidController.setTolerance(ArmConstants.kANGLE_TOLERANCE);
  }

  public static Arm getInstance() {
    if (single_instance == null)
      single_instance = new Arm();

    return single_instance;
  }

  public void setBrakeMotor() {
    Master_Motor.setNeutralMode(NeutralMode.Brake);
    Slave_Motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm encoder", getDegrees());
    SmartDashboard.putBoolean("Limit switch output", microSwitch.get());
    //SmartDashboard.putBoolean("Function output", GetIfPressed());
  }

  public void Elevate(double speed) {
    motorControllerGroup.set(-1 * speed);
  }

  public void Lower(double speed) {
    //if(encoder.get() > -20)
      motorControllerGroup.set(speed);
  }

  public void stop() {
    motorControllerGroup.set(0);
  }

  public double getDegrees() {
    if (encoder.getDistance() - ArmConstants.kENCODER_ERROR < -100)
      return encoder.getDistance();
    else
      return encoder.getDistance() - ArmConstants.kENCODER_ERROR;
  }

  public boolean GetIfPressed() {
    return !(microSwitch.get());
  }

  public double pidCalculate(double measurement, double setPoint) {
    armPidController.setSetpoint(setPoint);
    return armPidController.calculate(measurement, setPoint);
  }

  public boolean pidAtSetpoint() {
    return armPidController.atSetpoint();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}