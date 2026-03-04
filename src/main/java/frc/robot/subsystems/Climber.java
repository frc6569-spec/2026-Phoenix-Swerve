package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Climber extends SubsystemBase {

    private final SparkFlex climberMotor = new SparkFlex(59, MotorType.kBrushless);

    private final RelativeEncoder encoder = climberMotor.getEncoder();

    // position limits (example values)
    public static final double REST = 0;
    public static final double START = 10;
    public static final double UPPER_LIMIT = 60;
    public static final double LOWER_LIMIT = 0;

    public Climber() {

        encoder.setPosition(REST);

        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(80);

        climberMotor.configure(
            config,
            com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
            com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
        );
    }

    public void climbUp() {

        if (encoder.getPosition() < UPPER_LIMIT) {
            climberMotor.set(0.7);
        } else {
            climberMotor.set(0);
        }

    }

    public void climbDown() {

        if (encoder.getPosition() > LOWER_LIMIT) {
            climberMotor.set(-0.7);
        } else {
            climberMotor.set(0);
        }

    }

    public void stop() {
        climberMotor.set(0);
    }

    @Override
    public void periodic() {

        double position = encoder.getPosition();

        SmartDashboard.putNumber("Climber Position", position);
        SmartDashboard.putNumber("Climber Output", climberMotor.get());
        SmartDashboard.putNumber("Climber Upper Limit", UPPER_LIMIT);
        SmartDashboard.putNumber("Climber Lower Limit", LOWER_LIMIT);

    }
}