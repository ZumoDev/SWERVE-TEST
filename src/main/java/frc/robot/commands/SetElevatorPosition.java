package frc.robot.commands;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorPosition extends Command{
    private ElevatorSubsystem elevatorSub;
    private boolean aug;
    private int[] positions = {0, 2, 4, 6, 8, 10, 12};
    private static int index = 0;

    public SetElevatorPosition(ElevatorSubsystem elevatorSub, boolean aug) {
        this.elevatorSub = elevatorSub;
        this.aug = aug;
        addRequirements(elevatorSub);
    }

    @Override
    public void initialize() {
        if (aug) index++;
        else index--;

        if (index < 0) index = 0;
        if (index >= positions.length) index = positions.length - 1;

        elevatorSub.setElevatorPosition(positions[index] - 3.390136719);
        System.out.println("Elevator pos: "+positions[index]);
        System.out.println("Actual pos: "+elevatorSub.getElevatorPosition());
        //System.out.println("Desired pos: "+(elevatorSub.getElevatorPosition() - 3.390136719));
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return Math.abs(positions[index] - elevatorSub.getElevatorPosition()) <= 0.2;
    }
}
