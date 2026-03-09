package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class Slide extends SubsystemBase {

    private final TalonFX slideMotorLeader = new TalonFX(51);
    private final TalonFX slideMotorFollower = new TalonFX(52);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

    public static final double EXTENDED = 10.0;
    public static final double RETRACTED = 0.0;

    private double targetPosition = 0;

    public Slide() {

        // Reset encoder
        slideMotorLeader.setPosition(0);

        // Brake mode helps hold position
        slideMotorLeader.setNeutralMode(NeutralModeValue.Brake);
        slideMotorFollower.setNeutralMode(NeutralModeValue.Brake);

        var leaderConfig = slideMotorLeader.getConfigurator();
        var followerConfig = slideMotorFollower.getConfigurator();

        /* ----------------------------
         * Current Limits
         * ---------------------------- */
        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40.0;
        currentLimits.SupplyCurrentLimitEnable = true;

        leaderConfig.apply(currentLimits);
        followerConfig.apply(currentLimits);

        /* ----------------------------
         * PID + Feedforward
         * ---------------------------- */
        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();

        slot0Configs.kS = 0.25;   // static friction
        slot0Configs.kP = 3.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;

        // Gravity feedforward (helps hold slide up)
        slot0Configs.kG = 0.45;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        leaderConfig.apply(slot0Configs);

        /* ----------------------------
         * Motion Magic Settings
         * ---------------------------- */
        var motionMagicConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();

        motionMagicConfigs.MotionMagicCruiseVelocity = 20;
        motionMagicConfigs.MotionMagicAcceleration = 40;
        motionMagicConfigs.MotionMagicJerk = 0;

        leaderConfig.apply(motionMagicConfigs);

        /* ----------------------------
         * Soft Limits
         * ---------------------------- */
        var softLimits = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();

        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = EXTENDED;

        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = RETRACTED;

        leaderConfig.apply(softLimits);

        /* ----------------------------
         * Follower Setup
         * ---------------------------- */
        slideMotorFollower.setControl(
            new Follower(slideMotorLeader.getDeviceID(), MotorAlignmentValue.Aligned)
        );
    }

    /* ----------------------------
     * Slide Controls
     * ---------------------------- */

    public void setPosition(double rotations) {
        targetPosition = rotations;
        slideMotorLeader.setControl(motionMagicRequest.withPosition(rotations));
    }

    public void extend() {
        setPosition(EXTENDED);
    }

    public void retract() {
        setPosition(RETRACTED);
    }

    public void stop() {
        // Hold last commanded position
        slideMotorLeader.setControl(motionMagicRequest.withPosition(targetPosition));
    }

    public double getPosition() {
        return slideMotorLeader.getPosition().getValueAsDouble();
    }

    /* ----------------------------
     * Periodic Updates
     * ---------------------------- */

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Slide Position", getPosition());
        //SmartDashboard.putNumber("Slide Target", targetPosition);
    }
}