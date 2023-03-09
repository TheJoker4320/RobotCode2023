package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimeLight;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.PIDConstants;

public class Chassis extends SubsystemBase {
  public double startingAngle;

  public static boolean isToUsCollecting;
  public static boolean isAgainstUsCollecting;

  private WPI_TalonSRX rightMasterMotor;
  private WPI_TalonSRX rightSlaveMotor;
  private WPI_TalonSRX leftMasterMotor;
  private WPI_TalonSRX leftSlaveMotor;

  private MotorControllerGroup rightControllerGroup;
  private MotorControllerGroup leftControllerGroup;

  private Encoder rightEncoder;
  private Encoder leftEncoder;

  private DifferentialDrive differentialDrive;

  private PIDController pidControllerDriving;
  private PIDController pidControllerRamp;
  private PIDController pidControllerLimelight;

  private AHRS navXgyro;

  // -------------------------------------------
  // -----------------SingleTon-----------------
  // -------------------------------------------

  private static Chassis instantce = null;

  public static Chassis getInstantce() {
    if (instantce == null) {
      instantce = new Chassis();
    }
    return instantce;
  }

  // -------------------------------------------
  // -------------------------------------------
  // -------------------------------------------

  private Chassis() {
    isAgainstUsCollecting = false;
    isToUsCollecting = false;

    navXgyro = new AHRS();
    navXgyro.reset();

    pidControllerDriving = new PIDController(PIDConstants.kP_DRIVING, PIDConstants.kI_DRIVING, PIDConstants.kD_DRIVING);
    pidControllerDriving.setTolerance(PIDConstants.TOLERANCE_DRIVING);

    /*
     * Since the kp, i, d on the ramp are diffrent than regulary hence whise we will
     * have 2 pid controllers
     */

    pidControllerRamp = new PIDController(PIDConstants.kP_RAMP, PIDConstants.kI_RAMP, PIDConstants.kD_RAMP);
    pidControllerRamp.setTolerance(PIDConstants.TOLERANCE_RAMP);

    //-----

    pidControllerLimelight = new PIDController(LimeLightConstants.kP, LimeLightConstants.kI, LimeLightConstants.kD);
    pidControllerLimelight.setTolerance(LimeLightConstants.kTOLERANCE);

    rightMasterMotor = new WPI_TalonSRX(ChassisConstants.FRONT_RIGHT_MOTOR_PORT);
    rightMasterMotor.configFactoryDefault();

    rightSlaveMotor = new WPI_TalonSRX(ChassisConstants.BACK_RIGHT_MOTOR_PORT);
    rightSlaveMotor.configFactoryDefault();

    leftMasterMotor = new WPI_TalonSRX(ChassisConstants.FRONT_LEFT_MOTOR_PORT);
    leftMasterMotor.configFactoryDefault();

    leftSlaveMotor = new WPI_TalonSRX(ChassisConstants.BACK_LEFT_MOTOR_PORT);
    leftSlaveMotor.configFactoryDefault();

    rightEncoder = new Encoder(ChassisConstants.RIGHT_ENCODER_SCORE[0],
    ChassisConstants.RIGHT_ENCODER_SCORE[1]);
    leftEncoder = new Encoder(ChassisConstants.LEFT_ENCODER_SCORE[0],
    ChassisConstants.LEFT_ENCODER_SCORE[1]);

   // rightEncoder.setReverseDirection(true);
   // leftEncoder.setReverseDirection(true);
    
    rightEncoder.setDistancePerPulse(PIDConstants.TICKS_TO_METERS);
    leftEncoder.setDistancePerPulse(PIDConstants.TICKS_TO_METERS);

    rightControllerGroup = new MotorControllerGroup(rightMasterMotor, rightSlaveMotor);
    leftControllerGroup = new MotorControllerGroup(leftMasterMotor, leftSlaveMotor);
    leftControllerGroup.setInverted(true);
    differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

    resetEncoders();

    startingAngle = getAngle();

    setMotorsBrake();
  }

  public double getAngle() {
    return navXgyro.getRoll();
  }

  public void drive(final double forwardSpeed, final double turningSpeed) {
    double changableForwardSpeed = forwardSpeed;
    double changableRotationSpeed = turningSpeed;

    if (isToUsCollecting) {
      if (changableForwardSpeed < 0.7 && changableForwardSpeed >= 0.1)
        changableForwardSpeed = 0.7;
    }

    if (isAgainstUsCollecting) {
      if (changableForwardSpeed > 0.3)
        changableForwardSpeed = 0.3;
    }

    differentialDrive.arcadeDrive(changableForwardSpeed, changableRotationSpeed);
  }

  /*
   * We have noticed the the pid measurments on the ramp are different than
   * regularly hencewhise
   * we will need to know the type so we can know which ones to use.
   */

  public void setMotorsBrake() {
    rightMasterMotor.setNeutralMode(NeutralMode.Brake);
    rightSlaveMotor.setNeutralMode(NeutralMode.Brake);
    leftMasterMotor.setNeutralMode(NeutralMode.Brake);
    leftSlaveMotor.setNeutralMode(NeutralMode.Brake);

    /*rightMasterMotor.setSafetyEnabled(true);
    rightSlaveMotor.setSafetyEnabled(true);
    leftMasterMotor.setSafetyEnabled(true);
    leftSlaveMotor.setSafetyEnabled(true);*/
  }

  public double pidCalculate(double measurement, double setpoint, int type) {
    if (type == 1) {
      //SmartDashboard.putBoolean("got in type one", true);
      return pidControllerDriving.calculate(measurement, setpoint);
    } 
    else if (type == 2) {
      return pidControllerRamp.calculate(measurement, setpoint);
    }
    else {
      return pidControllerLimelight.calculate(measurement, setpoint);
    }
  }

  public boolean isAtSetpoint(int type) {
    if (type == 1) {
      return pidControllerDriving.atSetpoint();
    } else {
      return pidControllerRamp.atSetpoint();
    }
  }

  public void setPowerToMotors(double rightValue, double leftValue) {
    rightControllerGroup.set(rightValue);
    leftControllerGroup.set(leftValue);
  }

  public void stopDriving() {
    rightControllerGroup.set(0);
    leftControllerGroup.set(0);
  }

  public double getStartAngle() {
    return startingAngle;
  }

  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  public void resetEncoders() {
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public double getYawAngle() {
    return navXgyro.getYaw();
  }

  public void rotate(double rotationSpeed) {
    rightControllerGroup.set(1 * rotationSpeed);
    leftControllerGroup.set(-1 * rotationSpeed);
  }
  public double getLeftVolt(){
    return leftMasterMotor.getMotorOutputVoltage();
  }
  public double getRightVolt(){
    return rightMasterMotor.getMotorOutputVoltage();
  }

  public void lock() {
    rightControllerGroup.stopMotor();
    leftControllerGroup.stopMotor();
  }

  /*
   * 
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw", navXgyro.getYaw());
    SmartDashboard.putNumber("Pitch", navXgyro.getPitch());
    SmartDashboard.putNumber("Roll", navXgyro.getRoll());

    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());

    //SmartDashboard.putNumber("Right Encoder Raw", rightEncoder.getDistance() / rightEncoder.getDistancePerPulse());
    //SmartDashboard.putNumber("Left Encoder Raw", leftEncoder.getDistance() / leftEncoder.getDistancePerPulse());

    //SmartDashboard.putNumber("Angle error limelight", LimeLight.getInstance().FindAngleError());
  }
}