// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Claw extends SubsystemBase {
  DoubleSolenoid clawDoublePCM;
  static Claw instance;

  public static Claw getInstance()
  {
    if (instance == null)
      instance = new Claw();
      
    return instance;
  }

  private Claw() {
    clawDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.FIRST_CLAW_SOLENOID_PORT, PneumaticsConstants.SECOND_CLAW_SOLENOID_PORT);
    //clawDoublePCM.set(Value.kReverse);
  }

  public boolean getStateClaw()
  {
    if (clawDoublePCM.get() == Value.kForward)
      return true;
    else
      return false;
  }

  public void changeStateClaw(boolean state) {
    if (state)
      clawDoublePCM.set(Value.kForward);
    else
      clawDoublePCM.set(Value.kReverse);
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Current claw state:", getStateClaw());
  }
}
