package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class Feeder extends SubsystemBase {

    private final TalonFX feederMotor = new TalonFX(56); // change CAN ID if needed
    private final DutyCycleOut feederRequest = new DutyCycleOut(0);

    public Feeder() {

        // Optional current limit for protection
        var config = feederMotor.getConfigurator();

        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 30.0;
        currentLimits.SupplyCurrentLimitEnable = true;

        config.apply(currentLimits);
    }

    public void runFeeder() {
        double feederPower = 0.6;
        // Safety check: never allow negative power
        feederPower = Math.max(0.0, feederPower);

        feederMotor.setControl(feederRequest.withOutput(feederPower));
    }

    public void stopFeeder() {
        feederMotor.setControl(feederRequest.withOutput(0.0));
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Feeder Output", feederMotor.get());
    }
}