package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.LimelightHelpers;

public class RobotContainer {

    private final Feeder feeder = new Feeder();
    private final Intake intake = new Intake(feeder);
    private final Shooter shooter = new Shooter(feeder);
    private final Limelight limelight = new Limelight();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);

    public RobotContainer() {

        // Tag filter (safe Optional handling)
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            LimelightHelpers.SetFiducialIDFiltersOverride(
                "limelight",
                new int[]{21,24,25,26,18,27}
            );
        } else {
            LimelightHelpers.SetFiducialIDFiltersOverride(
                "limelight",
                new int[]{5,8,9,10,11,12}
            );
        }

        configureBindings();

    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double rawY = -driverXboxController.getLeftY();
                double rawX = -driverXboxController.getLeftX();
                double rawRot = -driverXboxController.getRightX();

                double squaredY = Math.copySign(rawY * rawY, rawY);
                double squaredX = Math.copySign(rawX * rawX, rawX);
                double squaredRot = Math.copySign(rawRot * rawRot, rawRot);

                double speedMultiplier =
                    driverXboxController.getLeftTriggerAxis() > 0.5 ? 0.5 : 1.0;

                double rot;

                boolean autoAim =
                    driverXboxController.getLeftTriggerAxis() > 0.5;

                if (autoAim && limelight.hasTarget()) {

                    double tx = limelight.getTX();
                    double kP = 0.03;

                    if (Math.abs(tx) < 1.0)
                        rot = 0;
                    else
                        rot = -tx * kP * MaxAngularRate;

                } else {

                    rot = squaredRot * MaxAngularRate;

                }

                return drive
                    .withVelocityX(squaredY * MaxSpeed * speedMultiplier)
                    .withVelocityY(squaredX * MaxSpeed * speedMultiplier)
                    .withRotationalRate(rot * speedMultiplier);
            })
        );

        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverXboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        driverXboxController.b().whileTrue(
            drivetrain.applyRequest(() ->
                point.withModuleDirection(
                    new Rotation2d(
                        -driverXboxController.getLeftY(),
                        -driverXboxController.getLeftX()
                    )
                )
            )
        );

        // SysId
        driverXboxController.back().and(driverXboxController.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        driverXboxController.back().and(driverXboxController.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        driverXboxController.start().and(driverXboxController.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        driverXboxController.start().and(driverXboxController.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset heading
        driverXboxController.back().and(driverXboxController.start())
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        ////////////////////////////////////////////////////////
        // Shooter test
        driverXboxController.y()
            .whileTrue(
                shooter.runEnd(
                    shooter::spinShooter,
                    shooter::stopShooter
                )
            );

        // Shooter + feeder
        driverXboxController.rightTrigger(.3)
            .whileTrue(
                Commands.runEnd(
                    () -> {
                        shooter.spinShooter();
                        feeder.runFeeder();
                    },
                    () -> {
                        shooter.stopShooter();
                        feeder.stopFeeder();
                    },
                    shooter,
                    feeder
                )
            );

        // Intake (Driver)
        driverXboxController.rightBumper()
            .debounce(0.1)
            .onTrue(intake.runOnce(intake::toggleExtendPushback));

        // Intake (Operator) ✅ NEW
        operatorXboxController.rightBumper()
            .debounce(0.1)
            .onTrue(intake.runOnce(intake::toggleExtendPushback));

        driverXboxController.leftBumper()
            .debounce(0.25)
            .onTrue(intake.runOnce(intake::retract));

        // Operator feeder
        operatorXboxController.x()
            .whileTrue(
                feeder.runEnd(
                    feeder::runFeeder,
                    feeder::stopFeeder
                )
            );
    }

    public void periodic() {
        SmartDashboard.putNumber("Limelight TX", limelight.getTX());
        SmartDashboard.putBoolean("Limelight Has Target", limelight.hasTarget());
        SmartDashboard.putNumber("LL Distance", limelight.getDistanceMeters());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}