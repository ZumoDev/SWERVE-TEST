package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStart extends Command {
    private IntakeSubsystem intakeSub;
    private double startTime;

    public IntakeStart(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        intakeSub.setIntakeSpeed(0.25);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intakeSub.setIntakeSpeed(0);
    }

    @Override 
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= 5;
    }
}
