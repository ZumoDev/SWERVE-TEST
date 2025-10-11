package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeKeep extends Command{
    private IntakeSubsystem intakeSub;

    public IntakeKeep(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intakeSub.setIntakeSpeed(0.05);
        intakeSub.setWristPos(1);
    }
}
