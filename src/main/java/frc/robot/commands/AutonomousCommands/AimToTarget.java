package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.LimeLight;
import frc.robot.Constants.LimeLightConstants;

public class AimToTarget extends CommandBase {

    private final Chassis chassis;
    double rotationSpeed;
    double angleError;
    double startingAngle;

    public AimToTarget(Chassis chassis) {
        this.chassis = chassis;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        angleError = LimeLight.GetErrorDegreeFromTarget();
        startingAngle = chassis.getYawAngle();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // aiming to the object//
        // rotating for a moment
        double rotationSpeed;
        rotationSpeed = chassis.pidCalculate(chassis.getYawAngle(), startingAngle + angleError, 3);

        chassis.rotate(rotationSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.drive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return chassis.getYawAngle() > (startingAngle + angleError) - LimeLightConstants.kTOLERANCE &&
               chassis.getYawAngle() < (startingAngle + angleError) + LimeLightConstants.kTOLERANCE;
    }

}