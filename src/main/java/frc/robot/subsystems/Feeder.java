package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;

public class Feeder extends SubsystemBase {

    private final TalonFX feederMotor = new TalonFX(56);
    private final VoltageOut feederRequest = new VoltageOut(0);

    private boolean intakeActive = false;
    private boolean overrideActive = false;

    public Feeder() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // 🔥 MUCH higher torque capability
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 80;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 120;

        // 🔥 IMPORTANT: allow full torque
        config.TorqueCurrent.PeakForwardTorqueCurrent = 120;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -120;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        feederMotor.getConfigurator().apply(config);
    }

    // Driver override (FULL POWER)
    public void runFeeder() {

        overrideActive = true;

        feederMotor.setControl(
            feederRequest.withOutput(12.0)
        );
    }

    public void stopFeeder() {

        overrideActive = false;

        feederMotor.setControl(
            feederRequest.withOutput(0)
        );
    }

    public void setIntakeActive(boolean active) {

        intakeActive = active;

        if (!active) {
            feederMotor.setControl(
                feederRequest.withOutput(0)
            );
        }
    }

    @Override
    public void periodic() {

        if (overrideActive) {

            SmartDashboard.putString("Feeder Mode", "Driver");
            return;
        }

        if (intakeActive) {

            // 🔥 constant strong feed
            feederMotor.setControl(
                feederRequest.withOutput(10.0) // ~83% power but stronger than duty cycle
            );

            SmartDashboard.putString("Feeder Mode", "Continuous");

        } else {

            feederMotor.setControl(
                feederRequest.withOutput(0)
            );

            SmartDashboard.putString("Feeder Mode", "Idle");
        }

        SmartDashboard.putNumber("Feeder Voltage", feederMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder Current", feederMotor.getStatorCurrent().getValueAsDouble());
    }
}