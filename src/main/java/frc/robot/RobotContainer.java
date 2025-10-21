// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeSpit;
import frc.robot.commands.IntakeStart;
import frc.robot.commands.IntakeWristSpeed;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);


    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController mvController = new CommandXboxController(0);
    //private final CommandXboxController tkController = new CommandXboxController(1);

    private final IntakeSubsystem intakeSub = new IntakeSubsystem();
    private final ElevatorSubsystem elevatorSub = new ElevatorSubsystem();
    private final PIDController limelightTurnPID = new PIDController(
        Constants.LimelightController.limelightkP, 
        Constants.LimelightController.limelightkI,
        Constants.LimelightController.limelightkD);

    //private final PIDController limelightElevatorPID = new PIDController(MaxAngularRate, MaxSpeed, MaxAngularRate);
    private final PIDController limelightElevatorPID = new PIDController(
        Constants.LimelightController.ElevatorController.kP, 
        Constants.LimelightController.ElevatorController.kI, 
        Constants.LimelightController.ElevatorController.kD);

    //private double lastTY = 0.0;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        /*mvController.leftBumper().whileFalse(drivetrain.applyRequest(() ->
            drive.withVelocityX(-mvController.getLeftY() * MaxSpeed * Constants.MovementReductions.velPercentage) // Drive forward with negative Y (forward)
                .withVelocityY(-mvController.getLeftX() * MaxSpeed * Constants.MovementReductions.velPercentage) // Drive left with negative X (left)
                .withRotationalRate(-mvController.getRightX() * MaxAngularRate * Constants.MovementReductions.turnPercentage) // Drive counterclockwise with negative X (left)
        )
        );

        mvController.leftBumper().whileTrue(
            drivetrain.applyRequest(() ->
                robotDrive.withVelocityX(-mvController.getLeftY() * MaxSpeed * Constants.MovementReductions.velPercentage) // Drive forward with negative Y (forward)
                    .withVelocityY(-mvController.getLeftX() * MaxSpeed * Constants.MovementReductions.velPercentage) // Drive left with negative X (left)
                    .withRotationalRate(-mvController.getRightX() * MaxAngularRate * Constants.MovementReductions.turnPercentage) // Drive counterclockwise with negative X (left)
            )
        );*/

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*mvController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        mvController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-mvController.getLeftY(), -mvController.getLeftX()))
        ));*/

        mvController.leftTrigger().whileTrue(new SetElevatorSpeed(elevatorSub, -0.075));
        mvController.rightTrigger().whileTrue(new SetElevatorSpeed(elevatorSub, 0.15));

        mvController.b().whileTrue(new IntakeStart(intakeSub));
        mvController.y().whileTrue(new IntakeSpit(intakeSub));

        mvController.x().whileTrue(new IntakeWristSpeed(intakeSub, 0.15));
        mvController.a().whileTrue(new IntakeWristSpeed(intakeSub, -0.075));

        mvController.back().and(mvController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        mvController.back().and(mvController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        mvController.start().and(mvController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        mvController.start().and(mvController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        mvController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return drivetrain.applyRequest(() -> {
          double tx = LimelightHelpers.getTX("limelight");
          double ty = LimelightHelpers.getTY("limelight");
          LimelightHelpers.setPipelineIndex("limelight", 1);
          
          if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getCurrentPipelineIndex("limelight") == 1) {
            double rotationOutput = limelightTurnPID.calculate(tx, Constants.LimelightController.limelightXError);
            rotationOutput = Math.max(Math.min(rotationOutput, MaxAngularRate), -MaxAngularRate);
            
            double elevatorOutput = limelightElevatorPID.calculate(Constants.LimelightController.ElevatorController.limelightYError, ty);
            elevatorOutput = Math.max(Math.min(elevatorOutput, 0.175), -0.175);
            
            /*System.out.println("tx: " + tx + " / rotOutput: " + rotationOutput);
            System.out.println("ty: " + ty + " / elevatorOutput: " + elevatorOutput);
            System.out.println("elevatorPos: " + elevatorSub.getElevatorPosition());*/
            
            if (Math.abs(elevatorOutput) >= 0.01) {
                if (elevatorSub.getElevatorPosition() <= 0.7 && ty < 1) {
                    elevatorSub.setElevatorSpeed(0.02);
                    System.out.println(true);
                } else if (elevatorSub.getElevatorPosition() >= 20 && ty > 1) {
                    System.out.println(true);
                    elevatorSub.setElevatorSpeed(0.02);
                }
                else {
                    elevatorSub.setElevatorSpeed(elevatorOutput);
                    System.out.println(false);
                }
            }
            
            return drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(rotationOutput);
          } 
          else {
            elevatorSub.setElevatorSpeed(0.025); //Constante para gravedad
            return drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0);
          }
        });
    }
}