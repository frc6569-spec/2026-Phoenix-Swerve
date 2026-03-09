package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Climber extends SubsystemBase {

    private final SparkFlex climberMotor = new SparkFlex(59, MotorType.kBrushless);
    private final RelativeEncoder encoder = climberMotor.getEncoder();

    // Smooth acceleration limiter
    private final SlewRateLimiter limiter = new SlewRateLimiter(1.5);

    // position limits
    public static final double REST = 0;
    public static final double START = 0;
    public static final double UPPER_LIMIT = 0;
    public static final double LOWER_LIMIT = -350;

    // motor outputs
    private static final double CLIMB_POWER = 0.6;
    private static final double HOLD_POWER = 0.00;

    // LIMIT OVERRIDE FLAG
    private boolean overrideLimits = false;

    public Climber() {

        encoder.setPosition(REST);

        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(100);

        climberMotor.configure(
            config,
            com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
            com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters
        );
    }

    /* ----------------------------
     * Override Control
     * ---------------------------- */

    public void setOverride(boolean enabled) {
        overrideLimits = enabled;
    }

    /* ----------------------------
     * Climb Up
     * ---------------------------- */

    public void climbUp() {

        if (overrideLimits || encoder.getPosition() < UPPER_LIMIT) {
            climberMotor.set(limiter.calculate(CLIMB_POWER));
        } else {
            stop();
        }

    }

    /* ----------------------------
     * Climb Down
     * ---------------------------- */

    public void climbDown() {

        if (overrideLimits || encoder.getPosition() > LOWER_LIMIT) {
            climberMotor.set(limiter.calculate(-CLIMB_POWER));
        } else {
            stop();
        }

    }

    /* ----------------------------
     * Stop / Hold
     * ---------------------------- */

    public void stop() {

        if (encoder.getPosition() < -0.2) {
            climberMotor.set(HOLD_POWER);
        } else {
            climberMotor.set(0);
        }

    }

    @Override
    public void periodic() {

        double position = encoder.getPosition();

        //SmartDashboard.putNumber("Climber Position", position);
        //SmartDashboard.putNumber("Climber Output", climberMotor.get());
        //SmartDashboard.putBoolean("Climber Override", overrideLimits);
        //SmartDashboard.putNumber("Climber Upper Limit", UPPER_LIMIT);
        //SmartDashboard.putNumber("Climber Lower Limit", LOWER_LIMIT);

    }
}