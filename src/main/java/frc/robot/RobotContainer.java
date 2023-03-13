// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.commands.ChangeClawState;
import frc.robot.commands.ChangeShift;
import frc.robot.commands.Collect;
import frc.robot.commands.DriveBySpeed;
import frc.robot.commands.Eject;
import frc.robot.commands.ElevateBySpeed;
import frc.robot.commands.LowerArmToSwitch;
import frc.robot.commands.LowerBySpeed;
import frc.robot.commands.RestrictDrivingAgainstRobot;
import frc.robot.commands.RestrictDrivingTowardsRobot;
import frc.robot.commands.AutonomousCommands.AimToTarget;
import frc.robot.commands.AutonomousCommands.CollectByTime;
import frc.robot.commands.AutonomousCommands.DriveByDistance;
import frc.robot.commands.AutonomousCommands.ReachConeAngle;
import frc.robot.commands.AutonomousCommands.ShiftPowerState;
import frc.robot.commands.AutonomousCommands.ShiftSpeedState;
import frc.robot.commands.AutonomousCommands.StabalzieOnRamp;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shifters;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Shifters shifters = Shifters.getInstance();
  private final Chassis chassis = Chassis.getInstantce();
  private final Arm arm = Arm.getInstance();
  private final Collector collector = Collector.getInstance();
  private final Claw claw = Claw.getInstance();

  private Joystick joystick = new Joystick(OperatorConstants.kDRIVING_PORT);
  private XboxController xboxController = new XboxController(OperatorConstants.kBUTTONS_PORT);

  public StabalzieOnRamp STABALZIE_ON_RAMP = new StabalzieOnRamp(Chassis.getInstantce());
  private final SendableChooser<String> autonomousChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  /**
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autonomousChooser.setDefaultOption("Stabalzing on ramp", PIDConstants.kOPTION_STABALIZE_RAMP);
    autonomousChooser.addOption("RIGHT Driving to collect game pieces", PIDConstants.kOPTION_DRIVE_TO_PIECES_RIGHT);
    autonomousChooser.addOption("LEFT Driving to collect game pieces", PIDConstants.kOPTION_DRIVE_TO_PIECES_LEFT);
    autonomousChooser.addOption("TEST", PIDConstants.kOPTION_TEST);
    SmartDashboard.putData("Autonomous chooser", autonomousChooser);

    chassis.setDefaultCommand(new DriveBySpeed(chassis, () -> joystick.getY(), () -> joystick.getZ()));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * 
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*
     * When button 12 on the joystick is pressed the shifter state's will change,
     * from open it will become closed
     * and from closed it will become open.
     */
    JoystickButton ChangeShifterState = new JoystickButton(joystick, OperatorConstants.SHIFTER_BUTTON);
    ChangeShifterState.onTrue(new ShiftSpeedState(shifters));
    ChangeShifterState.onFalse(new ShiftPowerState(shifters));


    /*
     * This lowers or elevates the arm depending on the intensity of the press on
     * the triggers,
     * the arms have a microswitch and if the microswitched is pressed the arm will
     * try to move in the opposite direction
     */
    JoystickButton lowerButton = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    lowerButton.whileTrue(new LowerBySpeed(arm));
    JoystickButton ElevateButton = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);
    ElevateButton.whileTrue(new ElevateBySpeed(arm));

    JoystickButton ReachConeHeight = new JoystickButton(xboxController, XboxController.Button.kLeftStick.value);
    ReachConeHeight.onTrue(new ReachConeAngle(arm));
    /*
     * Since the cones behave differently with the collector depending on the
     * directions it faces,
     * each one of these commands restricts the driving speed accordingly to the
     * cone's direction so the cone can enter 1successfully
     */

    JoystickButton ejectButton = new JoystickButton(xboxController, XboxController.Button.kY.value);
    ejectButton.whileTrue(new Eject(collector));
    JoystickButton collectButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
    collectButton.whileTrue(new Collect(collector));

    JoystickButton restrictDrivingTowards = new JoystickButton(joystick, OperatorConstants.COLLECT_FACING_BUTTON);
    restrictDrivingTowards.whileTrue(new RestrictDrivingTowardsRobot());
    JoystickButton restrictDrivingAgainst = new JoystickButton(joystick, OperatorConstants.COLLECT_AGAINST_BUTTON);
    restrictDrivingAgainst.whileTrue(new RestrictDrivingAgainstRobot());

    /*
     * When button B is pressed the claw state's will change, from open it will
     * become closed
     * and from closed it will become open.
     */
    JoystickButton changeClawState = new JoystickButton(xboxController, XboxController.Button.kB.value);
    changeClawState.onTrue(new ChangeClawState(claw));


    JoystickButton stabalizeOnRampButton = new JoystickButton(xboxController,  OperatorConstants.RAMP_BUTTON);
    stabalizeOnRampButton.onTrue(STABALZIE_ON_RAMP);

    //FrontCamera
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //We noticed that at the autonomous command it is best for the robot to first lift his arm before begining to drive towards the grid.
    //In order to not have any problems with the setting up of the robot we decided to assume he is set up close or even touching
    //the grid
    //meaning:
    //the final command will look something like this:
    //(liftArm).andThan(driveToGrid).andThan(wait).andThan(release).andThan(wait).andThan(driveToRamp/ToGamepieces) 
    
    if (autonomousChooser.getSelected() == PIDConstants.kOPTION_STABALIZE_RAMP) {
      claw.changeStateClaw(false);
      return (new DriveByDistance(chassis, 0.35, 0.35)).andThen(
              new ReachConeAngle(arm)).andThen(
              new DriveByDistance(chassis, -0.3, -0.3)).andThen(
              new WaitCommand(0.3)).andThen(
              new ChangeClawState(claw)).andThen(
              new WaitCommand(0.25)).andThen(
              new LowerArmToSwitch(arm)).andThen(
              new WaitCommand(0.25)).andThen(
              new DriveByDistance(chassis, PIDConstants.kDISTANCE_TO_RAMP, PIDConstants.kDISTANCE_TO_RAMP)).andThen(
              new WaitCommand(1)).andThen(
              new DriveByDistance(chassis, .55, .55)).andThen(
              new StabalzieOnRamp(chassis));
    }
    else if (autonomousChooser.getSelected() == PIDConstants.kOPTION_DRIVE_TO_PIECES_RIGHT) {
      claw.changeStateClaw(false);
      return (new DriveByDistance(chassis, 0.3, 0.3, 1.5)).andThen(
              new ReachConeAngle(arm)).andThen(
              new DriveByDistance(chassis, -0.4, -0.4, 1.5)).andThen(
              new WaitCommand(0.3)).andThen(
              new ChangeClawState(claw)).andThen(
              new WaitCommand(0.25)).andThen(
              new LowerArmToSwitch(arm)).andThen(
              new WaitCommand(0.25)).andThen(
              (new DriveByDistance(chassis, 4.7, 4.3 ).alongWith(
              new CollectByTime(3.5, collector))));
    }
    else if (autonomousChooser.getSelected() == PIDConstants.kOPTION_DRIVE_TO_PIECES_LEFT) {
      claw.changeStateClaw(false);
      return (new DriveByDistance(chassis, 0.3, 0.3, 1.5)).andThen(
              new ReachConeAngle(arm)).andThen(
              new DriveByDistance(chassis, -0.4, -0.4, 1.5)).andThen(
              new WaitCommand(0.3)).andThen(
              new ChangeClawState(claw)).andThen(
              new WaitCommand(0.25)).andThen(
              new LowerArmToSwitch(arm)).andThen(
              new WaitCommand(0.25)).andThen(
              (new DriveByDistance(chassis, 4.5, 4.7 ).alongWith(
              new CollectByTime(3.5, collector))));
    }
    else if (autonomousChooser.getSelected() == PIDConstants.kOPTION_TEST) {
      return (new DriveByDistance(chassis, 4.8, 4.35 ).alongWith(
        new CollectByTime(3.5, collector)));
    }
    else {
      return null;
    }
  }

  public void checkIfStabalizeInterruptPressed() {
    if (joystick.getRawButton(OperatorConstants.INTERRUPED_RAMP))
    {
      if (CommandScheduler.getInstance().isScheduled(STABALZIE_ON_RAMP))
      {
        CommandScheduler.getInstance().cancel(STABALZIE_ON_RAMP);
      }
    }
  }
  public double getLeftVolt(){
    return chassis.getLeftVolt();
  }
  public double getRightVolt(){
    return chassis.getRightVolt();
  }

  public Command getAutonomousPath(Boolean stabalzingOnRamp) {
    if (stabalzingOnRamp) {
      return (new DriveByDistance(chassis, -1, -1)
              .alongWith(new ReachConeAngle(arm)) //Paraller commands
              .andThen(new ChangeClawState(claw))
              .andThen(new DriveByDistance(chassis,PIDConstants.kDISTANCE_TO_RAMP, PIDConstants.kDISTANCE_TO_RAMP))
              .andThen(new StabalzieOnRamp(chassis)));
    }
    else {
      return (new DriveByDistance(chassis, -1, -1)
              .alongWith(new ReachConeAngle(arm)) //Paraller commands
              .andThen(new ChangeClawState(claw))
              .andThen(new DriveByDistance(chassis,PIDConstants.kDISTANCE_TO_GAMEPIECES - PIDConstants.kROBOT_LENGTH, PIDConstants.kDISTANCE_TO_GAMEPIECES - PIDConstants.kROBOT_LENGTH)));
    }
  }
  //TODO: ADD ENUM TO THE AUTONOMOUS PATH
  public void initOnSpeed(){
    shifters.changeStateShifter(false);
  }
  public void initOnForce() {
    shifters.changeStateShifter(true);
  }
}
