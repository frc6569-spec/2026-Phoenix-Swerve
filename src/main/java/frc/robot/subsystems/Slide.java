package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Slide extends SubsystemBase {
    public final TalonFX slideMotor = new TalonFX(51); // example CAN ID for the slide motor
    private final PositionVoltage positionRequest = new PositionVoltage(0.0); // default to retracted position

    public static final double EXTENDED = 25.0; // example position for extended
    public static final double RETRACTED = 0.0; // example position for retracted

    public Slide() {
        slideMotor.setPosition(0);

        var config = slideMotor.getConfigurator();

        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        slot0Configs.kP = 10.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;
        slot0Configs.kV = 0.0;
        slot0Configs.kS = 0.0; // overcome static friction

        config.apply(slot0Configs);
        var softLimits = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = EXTENDED;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = RETRACTED;

        config.apply(softLimits);
    }

    public void setPosition(double rotations) {
        slideMotor.setControl(positionRequest.withPosition(rotations));
    }
    
    public void extend() {
        setPosition(EXTENDED);
    }

    public void retract() {
        setPosition(RETRACTED);
    }

}
