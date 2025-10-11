package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX ElevatorL = new TalonFX(Constants.ElevatorMotorIDs.ElevatorL);
    private final TalonFX ElevatorR = new TalonFX(Constants.ElevatorMotorIDs.ElevatorR);
    private final PositionVoltage posvol = new PositionVoltage(0);
    private final TalonFXConfiguration pidConfig = new TalonFXConfiguration();

    public ElevatorSubsystem() {
        pidConfig.Slot0.kP = 0.5;
        pidConfig.Slot0.kG = 2;
        pidConfig.Slot0.kV = 0.0;
        pidConfig.Slot0.kA = 0.0;
        pidConfig.Slot0.kS = 5;
        pidConfig.Slot0.kI = 0.0;
        pidConfig.Slot0.kD = 0.0;
        pidConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        pidConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        ElevatorL.setControl(new Follower(ElevatorR.getDeviceID(), true));
        ElevatorR.getConfigurator().apply(pidConfig);
    }

    public void setElevatorPosition(double pos) {
        if (pos < ElevatorR.getPosition().getValueAsDouble()) ElevatorR.setControl(posvol.withPosition(pos)
        //.withFeedForward(0.05)
        );
        else ElevatorR.setControl(posvol.withPosition(pos)
        //.withFeedForward(-0.5)
        );
    }

    public double getElevatorPosition() {
        return ElevatorR.getPosition().getValueAsDouble();
    }
    
    public void setElevatorSpeed(double speed) {
        ElevatorR.set(speed);
    }
}
