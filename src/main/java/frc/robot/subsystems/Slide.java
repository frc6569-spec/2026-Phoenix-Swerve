package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class Slide extends SubsystemBase {
    private final TalonFX slideMotorLeader = new TalonFX(51); // CAN ID for the slide motor
    private final TalonFX slideMotorFollower = new TalonFX(52); // CAN ID for Follower Motor

    private final PositionVoltage positionRequest = new PositionVoltage(0.0); // default to retracted position

    public static final double EXTENDED = 5.0; // example position for extended
    public static final double RETRACTED = 0.0; // example position for retracted

    public Slide() {
        slideMotorLeader.setPosition(0);
        slideMotorFollower.setPosition(0);

        var config = slideMotorLeader.getConfigurator();
        var followerConfig = slideMotorFollower.getConfigurator();

        // Current limiting
        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40.0; // Max pull of XX amps from the battery.
        currentLimits.SupplyCurrentLimitEnable = true;

        config.apply(currentLimits);
        followerConfig.apply(currentLimits);

        var motorOutput = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive;

        followerConfig.apply(motorOutput);


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

        // Set follower motor to follow leader
        slideMotorFollower.setControl(new Follower(51, null));
    }

    public void setPosition(double rotations) {
        slideMotorLeader.setControl(positionRequest.withPosition(rotations));
    }
    
    public void extend() {
        setPosition(EXTENDED);
    }

    public void retract() {
        setPosition(RETRACTED);
    }

    public double getPosition() {
    return slideMotorLeader.getPosition().getValueAsDouble();
    }
    
    @Override
    public void periodic() {
    SmartDashboard.putNumber("Slide Position", slideMotorLeader.getPosition().getValueAsDouble());
    }
}
