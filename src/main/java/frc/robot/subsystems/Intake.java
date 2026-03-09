package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Intake extends SubsystemBase {

    // Extension motors
    private final TalonFX extensionLeader = new TalonFX(53);
    private final TalonFX extensionFollower = new TalonFX(54);

    // Intake shaft motor
    private final TalonFX intakeMotor = new TalonFX(55);

    private final PositionVoltage extensionRequest = new PositionVoltage(0);
    private final DutyCycleOut intakeRequest = new DutyCycleOut(0);

    // Positions
    public static final double EXTENDED = 25.0;
    public static final double RETRACTED = 0.0;

    private static final double INTAKE_POWER = .5;

    public Intake() {

        // Zero positions
        extensionLeader.setPosition(0);
        extensionFollower.setPosition(0);

        var leaderConfig = extensionLeader.getConfigurator();
        var followerConfig = extensionFollower.getConfigurator();
        var intakeConfig = intakeMotor.getConfigurator();

        // ----------------------------
        // Current limits
        // ----------------------------
        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40.0;
        currentLimits.SupplyCurrentLimitEnable = true;

        leaderConfig.apply(currentLimits);
        followerConfig.apply(currentLimits);
        intakeConfig.apply(currentLimits);

        // ----------------------------
        // Intake Ramp Rate (correct Phoenix 6 method)
        // ----------------------------
        var intakeFullConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        intakeFullConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 3.0; // 3 seconds ramp
        intakeConfig.apply(intakeFullConfig);

        // ----------------------------
        // Extension Follower
        // ----------------------------
        extensionFollower.setControl(
            new Follower(extensionLeader.getDeviceID(), MotorAlignmentValue.Aligned)
        );

        // ----------------------------
        // PID for extension
        // ----------------------------
        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        slot0Configs.kP = 10.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;

        leaderConfig.apply(slot0Configs);

        // ----------------------------
        // Soft limits
        // ----------------------------
        var softLimits = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = EXTENDED;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = RETRACTED;

        leaderConfig.apply(softLimits);
    }

    // ----------------------------
    // Extension Control
    // ----------------------------

    public void extendIntake() {
        extensionLeader.setControl(extensionRequest.withPosition(EXTENDED));
    }

    public void retractIntake() {
        extensionLeader.setControl(extensionRequest.withPosition(RETRACTED));
        stopIntake();
    }

    // ----------------------------
    // Intake Roller Control
    // ----------------------------

    public void runIntake() {
        intakeMotor.setControl(intakeRequest.withOutput(INTAKE_POWER));
    }

    public void stopIntake() {
        intakeMotor.setControl(intakeRequest.withOutput(0.0));
    }

    // ----------------------------
    // Check to make sure the intake is extended
    // ----------------------------

    public boolean isExtended() {
        double position = extensionLeader.getPosition().getValueAsDouble();
        return position >= EXTENDED * 0.9;
    }

    @Override
    public void periodic() {

        //double extensionPosition = extensionLeader.getPosition().getValueAsDouble();

        //SmartDashboard.putNumber("Intake Extension Position", extensionPosition);
        //SmartDashboard.putBoolean("Intake Extended", isExtended());
        //SmartDashboard.putNumber(
            //"Intake Roller Output",
            //intakeMotor.getDutyCycle().getValueAsDouble()
        //);
    }
}