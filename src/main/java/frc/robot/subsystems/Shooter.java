package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor = new TalonFX(54); // change CAN ID

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public static final double SHOOT_SPEED = 60; // rotations per second (example)

    public Shooter() {

        var config = shooterMotor.getConfigurator();

        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        slot0Configs.kP = 0.12;
        slot0Configs.kV = 0.12;

        config.apply(slot0Configs);
    }

    public void spinShooter() {
        shooterMotor.setControl(velocityRequest.withVelocity(SHOOT_SPEED));
    }

    public void stopShooter() {
        shooterMotor.setControl(velocityRequest.withVelocity(0));
    }

    public boolean atSpeed() {
        double speed = shooterMotor.getVelocity().getValueAsDouble();
        return speed > SHOOT_SPEED * 0.9;   // 90% of target
    }

    @Override
    public void periodic() {

    double currentSpeed = shooterMotor.getVelocity().getValueAsDouble();

    SmartDashboard.putNumber("Shooter Speed", currentSpeed);
    SmartDashboard.putBoolean("Shooter At Speed", atSpeed());
    SmartDashboard.putNumber("Shooter Target Speed", SHOOT_SPEED);
    }
}