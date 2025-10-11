package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePos extends Command{
    private IntakeSubsystem intakeSub;
    private boolean aug;
    private final int[] positions = {1, 3, 5};
    private static int index = 0;

    public IntakePos(IntakeSubsystem intakeSub, boolean aug) {
        this.intakeSub = intakeSub;
        this.aug = aug;
        //addRequirements(intakeSub);
    }

    @Override
    public void initialize() {
        if (aug) index++;
        else index--;

        if (index < 0) index = 0;
        if (index >= positions.length) index = positions.length - 1;

        intakeSub.setWristPos(positions[index]);
        System.out.println("Wrist pos: "+positions[index]);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
