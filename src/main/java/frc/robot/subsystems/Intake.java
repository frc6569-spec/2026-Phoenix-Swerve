package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;

public class Intake extends SubsystemBase {

    // Motors
    private final TalonFX extensionLeader = new TalonFX(53);
    private final TalonFX extensionFollower = new TalonFX(54);
    private final TalonFX intakeMotor = new TalonFX(55);

    // Control requests
    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut intakeRequest = new DutyCycleOut(0);

    // Gearbox constants
    private static final double GEAR_RATIO = 23.0;
    private static final double EXTEND_DEGREES = 110.0;

    private static final double EXTENDED =
        (EXTEND_DEGREES / 360.0) * GEAR_RATIO;

    private static final double RETRACTED = 0.0;

    // Intake states
    private enum IntakeState {
        RETRACTED,
        EXTENDING,
        EXTENDED,
        RETRACTING
    }

    private IntakeState state = IntakeState.RETRACTED;

    public Intake() {

        extensionLeader.setPosition(0);
        extensionFollower.setPosition(0);

        extensionLeader.setNeutralMode(NeutralModeValue.Brake);
        extensionFollower.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        // Leader configuration
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40;

        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.StatorCurrentLimit = 100;

        // Torque limits (prevents motor fighting)
        leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -70;

        // Torque ramp smoothing
        leaderConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.1;

        // PID
        leaderConfig.Slot0.kP = 20;
        leaderConfig.Slot0.kD = 0.2;

        // Motion Magic
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 25;
        leaderConfig.MotionMagic.MotionMagicAcceleration = 50;

        // Motion smoothing (S-curve)
        leaderConfig.MotionMagic.MotionMagicJerk = 200;

        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = EXTENDED;

        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RETRACTED;

        // Follower configuration (same but inverted)
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        followerConfig.CurrentLimits = leaderConfig.CurrentLimits;
        followerConfig.TorqueCurrent = leaderConfig.TorqueCurrent;
        followerConfig.ClosedLoopRamps = leaderConfig.ClosedLoopRamps;
        followerConfig.Slot0 = leaderConfig.Slot0;
        followerConfig.MotionMagic = leaderConfig.MotionMagic;
        followerConfig.SoftwareLimitSwitch = leaderConfig.SoftwareLimitSwitch;

        extensionLeader.getConfigurator().apply(leaderConfig);
        extensionFollower.getConfigurator().apply(followerConfig);

        SmartDashboard.putNumber("Intake Roller Speed", 0.6);
    }

    public void extend() {
        state = IntakeState.EXTENDING;
    }

    public void retract() {
        state = IntakeState.RETRACTING;
    }

    private void runIntake() {

        double speed = SmartDashboard.getNumber("Intake Roller Speed", 0.6);

        intakeMotor.setControl(
            intakeRequest.withOutput(speed)
        );
    }

    private void stopIntake() {

        intakeMotor.setControl(
            intakeRequest.withOutput(0)
        );
    }

    private boolean isExtended() {

        return extensionLeader
            .getPosition()
            .getValueAsDouble() >= EXTENDED * 0.95;
    }

    private boolean isRetracted() {

        return extensionLeader
            .getPosition()
            .getValueAsDouble() <= 0.05;
    }

    @Override
    public void periodic() {

        switch (state) {

            case EXTENDING:

                extensionLeader.setControl(
                    motionRequest.withPosition(EXTENDED)
                );

                extensionFollower.setControl(
                    motionRequest.withPosition(EXTENDED)
                );

                if (isExtended()) {
                    runIntake();
                    state = IntakeState.EXTENDED;
                }

            break;

            case RETRACTING:

                stopIntake();

                extensionLeader.setControl(
                    motionRequest.withPosition(RETRACTED)
                );

                extensionFollower.setControl(
                    motionRequest.withPosition(RETRACTED)
                );

                if (isRetracted()) {
                    state = IntakeState.RETRACTED;
                }

            break;

            case EXTENDED:
            break;

            case RETRACTED:
            break;
        }

        double leaderRot =
            extensionLeader.getPosition().getValueAsDouble();

        double followerRot =
            extensionFollower.getPosition().getValueAsDouble();

        double degrees =
            (leaderRot / GEAR_RATIO) * 360;

        SmartDashboard.putNumber("Leader Rotations", leaderRot);
        SmartDashboard.putNumber("Follower Rotations", followerRot);
        SmartDashboard.putNumber("Intake Degrees", degrees);
        SmartDashboard.putString("Intake State", state.toString());
    }
}