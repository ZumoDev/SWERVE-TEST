package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private TalonFX wrist = new TalonFX(Constants.IntakeMotorIDs.IntakeWrist);
    private TalonFX intake = new TalonFX(Constants.IntakeMotorIDs.IntakeMotor);
    private PositionVoltage posvol = new PositionVoltage(0);

    private TalonFXConfiguration pidConfig = new TalonFXConfiguration();

    public IntakeSubsystem() {
        pidConfig.Slot0.kG = 0;
        pidConfig.Slot0.kS = 4;
        pidConfig.Slot0.kP = 1.8;
        pidConfig.Slot0.kD = 0.0;
        pidConfig.Slot0.kI = 0.0;
        pidConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; //coseno de 0 (porque Slot0.kG = 0)
        pidConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        wrist.getConfigurator().apply(pidConfig);
    }

    public void setWristPos(double pos) {
        wrist.setControl(posvol.withPosition(pos));
    }

    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    public void setWristSpeed(double speed) {
        wrist.set(speed);
    }

    public double getWristPos() {
        return wrist.getPosition().getValueAsDouble();
    }
}
