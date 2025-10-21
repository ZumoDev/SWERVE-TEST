package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveSwerve extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    private final PIDController limelightPID;
    private final double maxAngularRate;
    private final String limelightName;

    public MoveSwerve(CommandSwerveDrivetrain drivetrain,
        SwerveRequest.FieldCentric driveRequest,
        PIDController limelightPID,
        double maxAngularRate,
        String limelightName) 
    {
        this.drivetrain = drivetrain;
        this.driveRequest = driveRequest;
        this.limelightPID = limelightPID;
        this.maxAngularRate = maxAngularRate;
        this.limelightName = limelightName;
    }

    @Override
    public void initialize() {
        SwerveRequest.FieldCentric request = driveRequest
        .withVelocityX(1)
        .withRotationalRate(0.0)
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    drivetrain.applyRequest(() -> request);
    System.out.println("FieldCentric request.");
    }
}
