package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Feeder extends SubsystemBase {

    private final TalonFX feederMotor = new TalonFX(56);
    private final DutyCycleOut feederRequest = new DutyCycleOut(0);

    private final Timer pulseTimer = new Timer();

    private static final double RUN_TIME = 1.0;
    private static final double WAIT_TIME = 2.5;

    private boolean intakeActive = false;
    private boolean overrideActive = false;
    private boolean pulseRunning = false;

    public Feeder() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;

        // Set feeder motor to COAST
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        feederMotor.getConfigurator().apply(config);

        pulseTimer.start();
    }

    // Driver override
    public void runFeeder() {

        overrideActive = true;

        feederMotor.setControl(
            feederRequest.withOutput(0.7)
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

            pulseRunning = false;
            pulseTimer.reset();
        }
    }

    private void pulseLogic() {

        double t = pulseTimer.get();

        if (!pulseRunning && t > WAIT_TIME) {

            feederMotor.setControl(
                feederRequest.withOutput(0.6)
            );

            pulseRunning = true;
            pulseTimer.reset();
        }

        if (pulseRunning && t > RUN_TIME) {

            feederMotor.setControl(
                feederRequest.withOutput(0)
            );

            pulseRunning = false;
            pulseTimer.reset();
        }
    }

    @Override
    public void periodic() {

        if (overrideActive) {

            SmartDashboard.putString("Feeder Mode", "Driver");
            return;
        }

        if (intakeActive) {

            pulseLogic();
            SmartDashboard.putString("Feeder Mode", "Pulse");

        } else {

            feederMotor.setControl(
                feederRequest.withOutput(0)
            );

            SmartDashboard.putString("Feeder Mode", "Idle");
        }

        SmartDashboard.putNumber("Feeder Output", feederMotor.get());
        SmartDashboard.putNumber("Pulse Timer", pulseTimer.get());
    }
}