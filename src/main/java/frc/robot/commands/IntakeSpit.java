package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpit extends Command{
    private IntakeSubsystem intakeSub;
    private double startTime;

    public IntakeSpit(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }

    @Override 
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        intakeSub.setIntakeSpeed(-0.3);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.setIntakeSpeed(0);
    }

    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= 5;
    }
}
