package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AprilTagTracker extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    private final PIDController limelightPID;
    private final double maxAngularRate;
    private final String limelightName;

    public AprilTagTracker(CommandSwerveDrivetrain drivetrain,
                           SwerveRequest.FieldCentric driveRequest,
                           PIDController limelightPID,
                           double maxAngularRate,
                           String limelightName) {
        this.drivetrain = drivetrain;
        this.driveRequest = driveRequest;
        this.limelightPID = limelightPID;
        this.maxAngularRate = maxAngularRate;
        this.limelightName = limelightName;
    }

    @Override
    public void initialize() {
        // Configura pipeline al inicio
        LimelightHelpers.setPipelineIndex(limelightName, 1);
    }

    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX(limelightName);
        double rotationOutput = 0.0;

        if (LimelightHelpers.getTV(limelightName)) {
            rotationOutput = limelightPID.calculate(tx, Constants.LimelightController.limelightXError);
            rotationOutput = Math.max(Math.min(rotationOutput, maxAngularRate), -maxAngularRate);

            System.out.println("tx: " + tx + " | rotOutput: " + rotationOutput);
        } 

        SwerveRequest.FieldCentric request = driveRequest
        .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rotationOutput)
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

        drivetrain.applyRequest(() -> request);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> driveRequest.withVelocityX(0.0)
                                                  .withVelocityY(0.0)
                                                  .withRotationalRate(0.0));
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
