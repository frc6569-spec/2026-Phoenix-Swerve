package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterLeader = new TalonFX(57);
    private final TalonFX shooterFollower = new TalonFX(58);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final Feeder feeder;

    // Default shooter speed
    private static final double DEFAULT_SHOOT_SPEED = 45; // Changed this from 50 for fundraiser

    // Idle speed (keeps flywheel spinning slowly)
    private static final double IDLE_SPEED = 10;

    // Shooter tolerance
    private static final double SPEED_TOLERANCE = 5;

    // Time shooter must remain stable before feeding
    private static final double SPEED_STABLE_TIME = 0.25;

    private final Timer speedTimer = new Timer();

    private boolean shooterEnabled = false;
    private boolean feederEnabled = false;

    public Shooter(Feeder feeder) {

        this.feeder = feeder;

        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

        leaderConfig.Slot0.kP = 0.18;
        leaderConfig.Slot0.kV = 0.24;

        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40;

        leaderConfig.Voltage.PeakForwardVoltage = 12.0;
        leaderConfig.Voltage.PeakReverseVoltage = -12.0;

        leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // IMPORTANT: Set coast mode
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        followerConfig.Slot0 = leaderConfig.Slot0;
        followerConfig.CurrentLimits = leaderConfig.CurrentLimits;
        followerConfig.Voltage = leaderConfig.Voltage;
        followerConfig.ClosedLoopRamps = leaderConfig.ClosedLoopRamps;

        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // IMPORTANT: Set coast mode
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterLeader.getConfigurator().apply(leaderConfig);
        shooterFollower.getConfigurator().apply(followerConfig);

        SmartDashboard.putNumber("Shooter Target RPS", DEFAULT_SHOOT_SPEED);

        // Start shooter idling
        idleShooter();
    }

    public void spinShooter() {

        shooterEnabled = true;
        feederEnabled = false;

        speedTimer.reset();
        speedTimer.start();

        double targetSpeed =
            SmartDashboard.getNumber("Shooter Target RPS", DEFAULT_SHOOT_SPEED);

        shooterLeader.setControl(
            velocityRequest.withVelocity(targetSpeed)
        );

        shooterFollower.setControl(
            velocityRequest.withVelocity(targetSpeed)
        );
    }

    // Used by Limelight auto aiming
    public void setSpeed(double speed) {

        shooterEnabled = true;
        feederEnabled = false;

        speedTimer.reset();
        speedTimer.start();

        SmartDashboard.putNumber("Shooter Target RPS", speed);

        shooterLeader.setControl(
            velocityRequest.withVelocity(speed)
        );

        shooterFollower.setControl(
            velocityRequest.withVelocity(speed)
        );
    }

    // Idle mode (keeps shooter spinning slowly)
    public void idleShooter() {

        shooterEnabled = false;
        feederEnabled = false;

        shooterLeader.setControl(
            velocityRequest.withVelocity(IDLE_SPEED)
        );

        shooterFollower.setControl(
            velocityRequest.withVelocity(IDLE_SPEED)
        );

        feeder.stopFeeder();
    }

    public void stopShooter() {

        shooterEnabled = false;
        feederEnabled = false;

        // Return to idle instead of stopping completely
        idleShooter();

        speedTimer.stop();
        speedTimer.reset();
    }

    public boolean atSpeed() {

        double targetSpeed =
            SmartDashboard.getNumber("Shooter Target RPS", DEFAULT_SHOOT_SPEED);

        double speed =
            shooterLeader.getVelocity().getValueAsDouble();

        return Math.abs(speed - targetSpeed) < SPEED_TOLERANCE;
    }

    public double getSpeedForDistance(double distance) {

        if (distance < 1.5) return 60;
        if (distance < 2.5) return 70;
        if (distance < 3.5) return 80;
        if (distance < 4.5) return 90;

        return 110;
    }

    @Override
    public void periodic() {

        double velocityRPS =
            shooterLeader.getVelocity().getValueAsDouble();

        double velocityRPM = velocityRPS * 60.0;

        double targetSpeed =
            SmartDashboard.getNumber("Shooter Target RPS", DEFAULT_SHOOT_SPEED);

        // Wait for shooter to stabilize before feeding
        if (shooterEnabled && atSpeed()) {

            if (!feederEnabled && speedTimer.hasElapsed(SPEED_STABLE_TIME)) {

                feederEnabled = true;
                feeder.runFeeder();

            }

        }

        // Stop feeder if shooter disabled
        if (!shooterEnabled) {

            feederEnabled = false;
            feeder.stopFeeder();
            speedTimer.reset();

        }

        SmartDashboard.putNumber("Shooter Velocity RPS", velocityRPS);
        SmartDashboard.putNumber("Shooter Velocity RPM", velocityRPM);
        SmartDashboard.putNumber("Shooter Target RPM", targetSpeed * 60);
        SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
        SmartDashboard.putBoolean("Feeder Enabled", feederEnabled);
        SmartDashboard.putNumber("Shooter Output", shooterLeader.get());
    }
}