package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterLeader = new TalonFX(57);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    // Good high-performance speed for Falcon shooters
    public static final double SHOOT_SPEED = 90;

    public Shooter() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID + Feedforward (helps motor reach speed faster)
        config.Slot0.kP = 0.18;
        config.Slot0.kV = 0.22;

        // Current limit to prevent brownouts
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        // Allow full voltage for max speed
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        shooterLeader.getConfigurator().apply(config);
    }

    public void spinShooter() {
        shooterLeader.setControl(velocityRequest.withVelocity(SHOOT_SPEED));
    }

    public void stopShooter() {
        shooterLeader.setControl(velocityRequest.withVelocity(0));
    }

    public boolean atSpeed() {
        double speed = shooterLeader.getVelocity().getValueAsDouble();
        return speed > SHOOT_SPEED * 0.9;
    }

    @Override
public void periodic() {

    double velocityRPS = shooterLeader.getVelocity().getValueAsDouble();
    double velocityRPM = velocityRPS * 60.0;

    SmartDashboard.putNumber("Shooter Velocity RPS", velocityRPS);
    SmartDashboard.putNumber("Shooter Velocity RPM", velocityRPM);
    SmartDashboard.putNumber("Shooter Target RPS", SHOOT_SPEED);
    SmartDashboard.putNumber("Shooter Target RPM", SHOOT_SPEED * 60);
    SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
    SmartDashboard.putNumber("Shooter Output", shooterLeader.get());
}
}