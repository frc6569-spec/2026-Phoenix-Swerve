package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class Intake extends SubsystemBase {

    // Extension motors
    private final TalonFX extensionLeader = new TalonFX(53);
    private final TalonFX extensionFollower = new TalonFX(54);

    // Intake roller
    private final TalonFX intakeMotor = new TalonFX(55);

    private final PositionVoltage extensionRequest = new PositionVoltage(0);
    private final DutyCycleOut intakeRequest = new DutyCycleOut(0);

    // Arm geometry
    private static final double GEAR_RATIO = 23.0;
    private static final double EXTEND_DEGREES = 110.0;

    public static final double EXTENDED =
        (EXTEND_DEGREES / 360.0) * GEAR_RATIO;

    public static final double RETRACTED = 0.0;

    // Intake roller speed (single variable like you requested)
    private static final double INTAKE_POWER = 0.5;

    public Intake() {

        extensionLeader.setPosition(0);
        extensionFollower.setPosition(0);

        var leaderConfig = extensionLeader.getConfigurator();
        var followerConfig = extensionFollower.getConfigurator();
        var intakeConfig = intakeMotor.getConfigurator();

        // Motor direction
        var motorOutput = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        motorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.apply(motorOutput);

        // Current limits
        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();

        currentLimits.SupplyCurrentLimit = 40;
        currentLimits.SupplyCurrentLimitEnable = true;

        leaderConfig.apply(currentLimits);
        followerConfig.apply(currentLimits);
        intakeConfig.apply(currentLimits);

        // Follower motor
        extensionFollower.setControl(
            new Follower(
                extensionLeader.getDeviceID(),
                MotorAlignmentValue.Opposed
            )
        );

        // PID
        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        slot0Configs.kP = 10.0;
        leaderConfig.apply(slot0Configs);

        // Soft limits
        var softLimits = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();

        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = EXTENDED;

        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = RETRACTED;

        leaderConfig.apply(softLimits);

        // SmartDashboard slider ONLY for extension speed
        SmartDashboard.putNumber("Extension Speed Limit", 0.25);
    }

    // Extend arm
    public void extendIntake() {

        extensionLeader.setControl(
            extensionRequest.withPosition(EXTENDED)
        );

    }

    // Retract arm
    public void retractIntake() {

        extensionLeader.setControl(
            extensionRequest.withPosition(RETRACTED)
        );

        stopIntake();
    }

    // Intake roller
    public void runIntake() {

        intakeMotor.setControl(
            intakeRequest.withOutput(INTAKE_POWER)
        );

    }

    public void stopIntake() {

        intakeMotor.setControl(
            intakeRequest.withOutput(0.0)
        );

    }

    public boolean isExtended() {

        double position = extensionLeader.getPosition().getValueAsDouble();
        return position >= EXTENDED * 0.9;

    }

    @Override
    public void periodic() {

        // Read extension speed slider
        double speedLimit =
            SmartDashboard.getNumber("Extension Speed Limit", 0.25);

        // Apply speed limit to extension motor only
        var outputLimit = new com.ctre.phoenix6.configs.MotorOutputConfigs();

        outputLimit.PeakForwardDutyCycle = speedLimit;
        outputLimit.PeakReverseDutyCycle = -speedLimit;

        extensionLeader.getConfigurator().apply(outputLimit);

        // Telemetry
        double motorRotations =
            extensionLeader.getPosition().getValueAsDouble();

        double armDegrees =
            (motorRotations / GEAR_RATIO) * 360.0;

        SmartDashboard.putNumber("Intake Motor Rotations", motorRotations);
        SmartDashboard.putNumber("Intake Arm Degrees", armDegrees);
        SmartDashboard.putBoolean("Intake Extended", isExtended());
    }
}