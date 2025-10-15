package frc.robot.commands;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorSpeed extends Command {
    private ElevatorSubsystem elevatorSub;
    private double speed;

    public SetElevatorSpeed(ElevatorSubsystem elevatorSub, double speed) {
        this.elevatorSub = elevatorSub;
        this.speed = speed;
    }

    @Override
    public void execute() {
        if (elevatorSub.getElevatorPosition() < 0 && speed < 0) elevatorSub.setElevatorSpeed(0.0);
        else if (elevatorSub.getElevatorPosition() > 20 && speed > 0) elevatorSub.setElevatorSpeed(0.02);
        else elevatorSub.setElevatorSpeed(speed);
        
        System.out.println("Elevator position: "+ elevatorSub.getElevatorPosition());
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSub.setElevatorSpeed(0.02);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
