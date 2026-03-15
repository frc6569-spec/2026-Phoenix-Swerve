package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterLeader = new TalonFX(57);
    private final TalonFX shooterFollower = new TalonFX(58);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private final Feeder feeder;

    // Default shooter speed
    private static final double DEFAULT_SHOOT_SPEED = 50;

    private boolean shooterEnabled = false;

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

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        followerConfig.Slot0 = leaderConfig.Slot0;
        followerConfig.CurrentLimits = leaderConfig.CurrentLimits;
        followerConfig.Voltage = leaderConfig.Voltage;
        followerConfig.ClosedLoopRamps = leaderConfig.ClosedLoopRamps;

        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shooterLeader.getConfigurator().apply(leaderConfig);
        shooterFollower.getConfigurator().apply(followerConfig);

        // Initialize dashboard control
        SmartDashboard.putNumber("Shooter Target RPS", DEFAULT_SHOOT_SPEED);
    }

    public void spinShooter() {

        shooterEnabled = true;

        double targetSpeed =
            SmartDashboard.getNumber("Shooter Target RPS", DEFAULT_SHOOT_SPEED);

        shooterLeader.setControl(
            velocityRequest.withVelocity(targetSpeed)
        );

        shooterFollower.setControl(
            velocityRequest.withVelocity(targetSpeed)
        );
    }

    // NEW METHOD (used by Limelight auto aiming)
    public void setSpeed(double speed) {

        shooterEnabled = true;

        SmartDashboard.putNumber("Shooter Target RPS", speed);

        shooterLeader.setControl(
            velocityRequest.withVelocity(speed)
        );

        shooterFollower.setControl(
            velocityRequest.withVelocity(speed)
        );
    }

    public void stopShooter() {

        shooterEnabled = false;

        shooterLeader.setControl(
            velocityRequest.withVelocity(0)
        );

        shooterFollower.setControl(
            velocityRequest.withVelocity(0)
        );

        feeder.stopFeeder();
    }

    public boolean atSpeed() {

        double targetSpeed =
            SmartDashboard.getNumber("Shooter Target RPS", DEFAULT_SHOOT_SPEED);

        double speed =
            shooterLeader.getVelocity().getValueAsDouble();

        return speed > targetSpeed * 0.9;
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

        // Automatic feeding
        if (shooterEnabled && atSpeed()) {
            feeder.runFeeder();
        }

        if (!shooterEnabled) {
            feeder.stopFeeder();
        }

        SmartDashboard.putNumber("Shooter Velocity RPS", velocityRPS);
        SmartDashboard.putNumber("Shooter Velocity RPM", velocityRPM);
        SmartDashboard.putNumber("Shooter Target RPM", targetSpeed * 60);
        SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
        SmartDashboard.putNumber("Shooter Output", shooterLeader.get());
    }
}