package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeWristSpeed extends Command {
    private double speed;
    private IntakeSubsystem intakeSub;

    public IntakeWristSpeed(IntakeSubsystem intakeSub, double speed) {
        this.intakeSub = intakeSub;
        this.speed = speed;
        addRequirements(intakeSub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (intakeSub.getWristPos() < 0.2 && speed < 0) intakeSub.setWristSpeed(0.0);
        else if (intakeSub.getWristPos() > 7 && speed > 0) intakeSub.setWristSpeed(0.0175);
        else intakeSub.setWristSpeed(speed);
        
        System.out.println("Wrist position: "+ intakeSub.getWristPos());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.setWristSpeed(0.0175);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
