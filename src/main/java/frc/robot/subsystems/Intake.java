package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {

    // Extension motors
    private final TalonFX extensionLeader = new TalonFX(53);
    private final TalonFX extensionFollower = new TalonFX(54);

    // Intake shaft motor
    private final TalonFX intakeMotor = new TalonFX(55);

    private final PositionVoltage extensionRequest = new PositionVoltage(0);
    private final DutyCycleOut intakeRequest = new DutyCycleOut(0);

    // Positions
    public static final double EXTENDED = 10.0;
    public static final double RETRACTED = 0.0;

    private static final double INTAKE_POWER = 0.6;

    public Intake() {

        extensionLeader.setPosition(0);
        extensionFollower.setPosition(0);

        var leaderConfig = extensionLeader.getConfigurator();
        var followerConfig = extensionFollower.getConfigurator();
        var intakeConfig = intakeMotor.getConfigurator();

        var currentLimits = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = 40.0;
        currentLimits.SupplyCurrentLimitEnable = true;

        leaderConfig.apply(currentLimits);
        followerConfig.apply(currentLimits);
        intakeConfig.apply(currentLimits);

        // follower motor
        extensionFollower.setControl(new Follower(53, null));

        //PID
        var slot0Configs = new com.ctre.phoenix6.configs.Slot0Configs();
        slot0Configs.kP = 10.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;

        leaderConfig.apply(slot0Configs);

        // Soft limits
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

        // turn intake off when retracting
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

        double extensionPosition = extensionLeader.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Intake Extension Position", extensionPosition);
        SmartDashboard.putBoolean("Intake Extended", isExtended());
        SmartDashboard.putNumber("Intake Roller Output",intakeMotor.getDutyCycle().getValueAsDouble());
    }
}