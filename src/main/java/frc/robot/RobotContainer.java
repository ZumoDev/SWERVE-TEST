// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AprilTagTracker;
import frc.robot.commands.IntakePos;
import frc.robot.commands.IntakeSpit;
import frc.robot.commands.IntakeStart;
import frc.robot.commands.IntakeWristSpeed;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetElevatorSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    /*
     * VELOCIDADES DEL MOTOR
     * Por defecto, "MaxSpeed" utiliza TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); para calcular la velocidad en base al
     * gearing y diámetro de la rueda. 
     * Por defecto, "MaxAngularRate" gira a 0.75 rotaciones por segundo, o ~2.36 radianes por segundo.
     */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    //FieldCentric orienta el "frente" del robot conforme a la zona de juego.
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //El deadband es la "zona muerta" del joystick; donde ya no se recibirán más inputs.
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    //RobotCentric orienta el frente del robot conforme a la 
            
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

    private double lastTY = 0.0;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
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
        /*drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-mvController.getLeftY() * MaxSpeed * Constants.MovementReductions.velPercentage) // Drive forward with negative Y (forward)
                    .withVelocityY(-mvController.getLeftX() * MaxSpeed * Constants.MovementReductions.velPercentage) // Drive left with negative X (left)
                    .withRotationalRate(-mvController.getRightX() * MaxAngularRate * Constants.MovementReductions.turnPercentage) // Drive counterclockwise with negative X (left)
            )
        );*/

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*mvController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        mvController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-mvController.getLeftY(), -mvController.getLeftX()))
        ));*/

        /*mvController.y().whileTrue(new IntakeStart(intakeSub));
        mvController.x().whileTrue(new IntakePos(intakeSub, false));
        mvController.b().whileTrue(new IntakePos(intakeSub, true));
        mvController.a().whileTrue(new IntakeSpit(intakeSub));

        mvController.povUp().whileTrue(new SetElevatorPosition(elevatorSub,true));
        mvController.povDown().whileTrue(new SetElevatorPosition(elevatorSub, false));*/

        mvController.leftTrigger().whileTrue(new SetElevatorSpeed(elevatorSub, -0.075));
        mvController.rightTrigger().whileTrue(new SetElevatorSpeed(elevatorSub, 0.15));

        mvController.b().whileTrue(new IntakeStart(intakeSub));
        mvController.y().whileTrue(new IntakeSpit(intakeSub));

        mvController.x().whileTrue(new IntakeWristSpeed(intakeSub, 0.15));
        mvController.a().whileTrue(new IntakeWristSpeed(intakeSub, -0.075));

        // Run SysId routines when holding back/start and X/Y.
        //////// Note that each routine should be run exactly once in a single log. <<<< IMPORTANTE
        /*
         * 1 y 2: Mide la respuesta al entorno del sistema mediante una aceleración rápida y un voltaje alto.
         * 3 y 4: Obtiene información sobre la fricción estática y dinámica con una aceleración lenta y voltaje bajo.
         * Ambos sirven para obtener valores que se pueden modificar en la PID, para cálculos más precisos.
         */
        mvController.back().and(mvController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward)); //1
        mvController.back().and(mvController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse)); //2
        mvController.start().and(mvController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward)); //3
        mvController.start().and(mvController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); //4

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
            
            System.out.println("tx: " + tx + " / rotOutput: " + rotationOutput);
            System.out.println("ty: " + ty + " / elevatorOutput: " + elevatorOutput);
            System.out.println("elevatorPos: " + elevatorSub.getElevatorPosition());
            
            if (Math.abs(elevatorOutput) >= 0.01) {
                if (elevatorSub.getElevatorPosition() <= 0.7 && ty < 1) {
                    System.out.println(true);
                } else {
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