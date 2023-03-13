package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Shifters extends SubsystemBase {
  private static DoubleSolenoid shifterSolenoid;
  static Shifters pneumaticsSubsystem = null;

  public static Shifters getInstance()
  {
    if (pneumaticsSubsystem == null)
      pneumaticsSubsystem = new Shifters();
      
    return pneumaticsSubsystem;
  }

  private Shifters() {
    shifterSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.FIRST_SHIFTER_SOLENOID_PORT, PneumaticsConstants.SECOND_SHIFTER_SOLENOID_PORT);
  }

  public boolean getStateShifter()
  {
    if (shifterSolenoid.get() == Value.kForward)
      return true;
    else
      return false;
  }

  public void changeStateShifter(boolean state) {
    if (state)
      shifterSolenoid.set(Value.kForward);
    else
      shifterSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Current shift state:", getStateShifter());
  }
}