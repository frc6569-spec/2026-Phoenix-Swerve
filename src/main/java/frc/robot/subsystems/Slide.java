package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Slide extends SubsystemBase {

    private final TalonFX slideMotorLeader = new TalonFX(51);
    private final TalonFX slideMotorFollower = new TalonFX(52);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

    public static final double EXTENDED = 25.0;
    public static final double RETRACTED = 0.0;

    public Slide() {

        slideMotorLeader.setPosition(0);
        slideMotorFollower.setPosition(0);

        var config = slideMotorLeader.getConfigurator();
        var followerConfig = slideMotorFollower.getConfigurator();

        // ----------------------------
        // Current limiting
        // ----------------------------
        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40.0;
        currentLimits.SupplyCurrentLimitEnable = true;

        config.apply(currentLimits);
        followerConfig.apply(currentLimits);

        // ----------------------------
        // PID
        // ----------------------------
        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        slot0Configs.kP = 10.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;

        config.apply(slot0Configs);

        // ----------------------------
        // Motion Magic Settings (THIS IS THE KEY)
        // ----------------------------
        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 10;   // rotations per second
        motionMagicConfigs.MotionMagicAcceleration = 20;     // rotations per sec^2
        motionMagicConfigs.MotionMagicJerk = 0;              // 0 = trapezoidal profile

        config.apply(motionMagicConfigs);

        // ----------------------------
        // Soft limits
        // ----------------------------
        var softLimits = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = EXTENDED;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = RETRACTED;

        config.apply(softLimits);

        // ----------------------------
        // Follower
        // ----------------------------
        slideMotorFollower.setControl(
            new Follower(
                slideMotorLeader.getDeviceID(),
                MotorAlignmentValue.Aligned
            )
        );
    }

    public void setPosition(double rotations) {
        slideMotorLeader.setControl(motionMagicRequest.withPosition(rotations));
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
        SmartDashboard.putNumber(
            "Slide Position",
            slideMotorLeader.getPosition().getValueAsDouble()
        );
    }
}