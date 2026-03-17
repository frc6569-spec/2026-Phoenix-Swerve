package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;

public class Intake extends SubsystemBase {

    private final Feeder feeder;

    private final TalonFX extensionLeader = new TalonFX(53);
    private final TalonFX extensionFollower = new TalonFX(54);
    private final TalonFX intakeMotor = new TalonFX(55);
    private final TalonFX assistMotor = new TalonFX(51);

    private final MotionMagicVoltage motionRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage intakeVelocity = new VelocityVoltage(0);
    private final VoltageOut intakeRequest = new VoltageOut(0);
    private final VoltageOut assistRequest = new VoltageOut(0);

    private static final double GEAR_RATIO = 23.0;
    private static final double DEFAULT_EXTEND_DEGREES = 135.0;

    private static final double RETRACTED = 0.0;

    // Kraken max speed
    private static final double MAX_RPS = 100.0;

    private boolean toggleExtended = false;

    private enum IntakeState {
        RETRACTED,
        EXTENDING,
        EXTENDED,
        RETRACTING
    }

    private IntakeState state = IntakeState.RETRACTED;

    public Intake(Feeder feeder) {

        this.feeder = feeder;

        extensionLeader.setPosition(0);
        extensionFollower.setPosition(0);

        extensionFollower.setPosition(
            extensionLeader.getPosition().getValueAsDouble()
        );

        extensionLeader.setNeutralMode(NeutralModeValue.Brake);
        extensionFollower.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        assistMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.DutyCycleNeutralDeadband = 0.02;

        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40;

        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.StatorCurrentLimit = 90;

        leaderConfig.TorqueCurrent.PeakForwardTorqueCurrent = 65;
        leaderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -65;

        leaderConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.12;

        leaderConfig.Slot0.kP = 20;
        leaderConfig.Slot0.kD = 0.2;
        leaderConfig.Slot0.kG = 0.15;

        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 25;
        leaderConfig.MotionMagic.MotionMagicAcceleration = 40;
        leaderConfig.MotionMagic.MotionMagicJerk = 180;

        double extended =
            (DEFAULT_EXTEND_DEGREES / 360.0) * GEAR_RATIO;

        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = extended;

        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RETRACTED;

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerConfig.CurrentLimits = leaderConfig.CurrentLimits;
        followerConfig.TorqueCurrent = leaderConfig.TorqueCurrent;
        followerConfig.ClosedLoopRamps = leaderConfig.ClosedLoopRamps;
        followerConfig.Slot0 = leaderConfig.Slot0;
        followerConfig.MotionMagic = leaderConfig.MotionMagic;

        extensionLeader.getConfigurator().apply(leaderConfig);
        extensionFollower.getConfigurator().apply(followerConfig);

        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 80;

        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 120;

        intakeConfig.Slot0.kP = 0.12;
        intakeConfig.Slot0.kV = 0.12;

        intakeMotor.getConfigurator().apply(intakeConfig);

        TalonFXConfiguration assistConfig = new TalonFXConfiguration();

        assistConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        assistConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        assistConfig.CurrentLimits.SupplyCurrentLimit = 60;

        assistConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        assistConfig.CurrentLimits.StatorCurrentLimit = 100;

        assistMotor.getConfigurator().apply(assistConfig);

        SmartDashboard.putNumber("Intake Roller Speed", 1.0);
        SmartDashboard.putNumber("Intake Assist Speed", 0.25);
        SmartDashboard.putNumber("Intake Extend Degrees", DEFAULT_EXTEND_DEGREES);
        SmartDashboard.putNumber("Intake Pushback Degrees", 75.0);
    }

    private void setArmPosition(double position) {

        extensionLeader.setControl(
            motionRequest.withPosition(position)
        );

        extensionFollower.setControl(
            motionRequest.withPosition(position)
        );
    }

    public void toggleExtendPushback() {

        double extendDegrees =
            SmartDashboard.getNumber("Intake Extend Degrees", DEFAULT_EXTEND_DEGREES);

        double pushbackDegrees =
            SmartDashboard.getNumber("Intake Pushback Degrees", 75.0);

        double EXTENDED =
            (extendDegrees / 360.0) * GEAR_RATIO;

        double PUSHBACK =
            (pushbackDegrees / 360.0) * GEAR_RATIO;

        if (!toggleExtended) {

            state = IntakeState.EXTENDING;
            toggleExtended = true;

        } else {

            setArmPosition(PUSHBACK);

            runIntake();
            feeder.setIntakeActive(true);

            toggleExtended = false;
        }
    }

    public void retract() {

        state = IntakeState.RETRACTING;
        toggleExtended = false;
    }

    private void runAssist() {

        double speed =
            SmartDashboard.getNumber("Intake Assist Speed", 0.25);

        assistMotor.setControl(
            assistRequest.withOutput(speed * 6.0)
        );
    }

    private void stopAssist() {

        assistMotor.setControl(
            assistRequest.withOutput(0)
        );
    }

    private void runIntake() {

        double speed =
            SmartDashboard.getNumber("Intake Roller Speed", 1.0);

        intakeMotor.setControl(
            intakeVelocity.withVelocity(MAX_RPS * speed)
        );

        runAssist();
    }

    private void stopIntake() {

        intakeMotor.setControl(
            intakeRequest.withOutput(0)
        );

        stopAssist();
    }

    private boolean isExtended(double extended) {

        return extensionLeader
            .getPosition()
            .getValueAsDouble() >= extended * 0.95;
    }

    private boolean isRetracted() {

        return extensionLeader
            .getPosition()
            .getValueAsDouble() <= 0.05;
    }

    public void goToPushback() {

        double pushbackDegrees =
            SmartDashboard.getNumber("Intake Pushback Degrees", 75.0);

        double PUSHBACK =
            (pushbackDegrees / 360.0) * GEAR_RATIO;

        // Go directly to pushback position
        setArmPosition(PUSHBACK);

        // Run intake + feeder
        runIntake();
        feeder.setIntakeActive(true);

        // Force correct state
        state = IntakeState.EXTENDED;
        toggleExtended = false;
    }

    @Override
    public void periodic() {

        double extendDegrees =
            SmartDashboard.getNumber("Intake Extend Degrees", DEFAULT_EXTEND_DEGREES);

        double EXTENDED =
            (extendDegrees / 360.0) * GEAR_RATIO;

        switch (state) {

            case EXTENDING:

                setArmPosition(EXTENDED);

                if (isExtended(EXTENDED)) {

                    runIntake();
                    feeder.setIntakeActive(true);
                    state = IntakeState.EXTENDED;
                }

            break;

            case RETRACTING:

                stopIntake();
                feeder.setIntakeActive(false);

                setArmPosition(RETRACTED);

                if (isRetracted()) {
                    state = IntakeState.RETRACTED;
                }

            break;

            case EXTENDED:

                runIntake();
                feeder.setIntakeActive(true);

            break;

            case RETRACTED:
            break;
        }
    }
}