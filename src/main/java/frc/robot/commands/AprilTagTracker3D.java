package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AprilTagTracker3D extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    private final PIDController limelightPID3D;
    private final double maxAngularRate;
    private final double maxSpeed;
    private final String limelightName; 

    public AprilTagTracker3D(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric driveRequest, PIDController limelightPID3D, double maxAngularRate, double maxSpeed, String limelightName) {
        this.drivetrain = drivetrain;
        this.driveRequest = driveRequest;
        this.limelightPID3D = limelightPID3D;
        this.maxAngularRate = maxAngularRate;
        this.maxSpeed = maxSpeed;
        this.limelightName = limelightName;

        addRequirements(drivetrain); 
    }

    @Override
    public void execute() {
        double[] targetLocation = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
        double tx = targetLocation[0];
        double tz = targetLocation[2];
        double yaw = targetLocation[5];

        LimelightHelpers.setPipelineIndex(limelightName, 1);
        SwerveRequest.FieldCentric newRequest;

        if (LimelightHelpers.getTV(limelightName)) {
            double xOutput = limelightPID3D.calculate(tx, Constants.Limelight3DController.limelight3DXError);
            double zOutput = limelightPID3D.calculate(tz, Constants.Limelight3DController.limelight3DZError);
            double rotationOutput = limelightPID3D.calculate(yaw, Constants.Limelight3DController.limelight3DYaw);

            xOutput = Math.max(Math.min(xOutput, maxSpeed), -maxSpeed);
            zOutput = Math.max(Math.min(zOutput, maxSpeed), -maxSpeed);
            rotationOutput = Math.max(Math.min(rotationOutput, maxAngularRate), -maxAngularRate);

            System.out.println("tx: " + tx + " | rotOutput: " + rotationOutput);

            newRequest = driveRequest.withVelocityX(zOutput)
                .withVelocityY(xOutput)
                .withRotationalRate(rotationOutput);
        } else {
            newRequest = driveRequest.withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0);
        }

        drivetrain.applyRequest(() -> newRequest);
    }
}
