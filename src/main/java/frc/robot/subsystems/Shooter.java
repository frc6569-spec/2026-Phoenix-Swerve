package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;


public class Shooter extends SubsystemBase {

    private final TalonFX shooterLeader = new TalonFX(57); // change CAN ID
    private final TalonFX shooterFollower = new TalonFX(58);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public static final double SHOOT_SPEED = 60; // rotations per second (example)

    public Shooter() {
            
        var config = shooterLeader.getConfigurator();

        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        slot0Configs.kP = 0.12;
        slot0Configs.kV = 0.12;

        config.apply(slot0Configs);

        // follower motor mirrors the leader
        shooterFollower.setControl(new Follower(57, null));
    }

    public void spinShooter() {
        shooterLeader.setControl(velocityRequest.withVelocity(SHOOT_SPEED));
    }

    public void stopShooter() {
        shooterLeader.setControl(velocityRequest.withVelocity(0));
    }

    public boolean atSpeed() {
        double speed = shooterLeader.getVelocity().getValueAsDouble();
        return speed > SHOOT_SPEED * 0.9;   // 90% of target
    }

    @Override
    public void periodic() {

    double currentSpeed = shooterLeader.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("Shooter Speed", currentSpeed);
    SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
    SmartDashboard.putNumber("Shooter Target Speed", SHOOT_SPEED);
    }
}