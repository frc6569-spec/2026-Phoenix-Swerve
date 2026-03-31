package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

    // Subsystems
    private final Feeder feeder = new Feeder();
    private final Intake intake = new Intake(feeder);
    private final Shooter shooter = new Shooter(feeder);
    private final Limelight limelight = new Limelight();
    private final SendableChooser<Command> autoChooser;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Drive config
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);

    public RobotContainer() {

        // Named commands for auto
        NamedCommands.registerCommand(
            "Intake Pushback Position",
            Commands.runOnce(() -> intake.toggleExtendPushback(), intake)
        );

        NamedCommands.registerCommand(
            "Shoot",
            Commands.sequence(
                Commands.runOnce(() -> shooter.setSpeed(50), shooter),
                Commands.runOnce(() -> shooter.spinShooter(), shooter),
                Commands.waitSeconds(1.5),
                Commands.runEnd(
                    () -> feeder.runFeeder(),
                    () -> feeder.stopFeeder(),
                    feeder
                ).withTimeout(10.0),
                Commands.runOnce(() -> shooter.stopShooter(), shooter)
            )
        );

        // Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {

        // ✅ STABLE DRIVE (NO TWITCH)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double rawY = -driverXboxController.getLeftY();
                double rawX = -driverXboxController.getLeftX();
                double rawRot = -driverXboxController.getRightX();

                // Deadband
                if (Math.abs(rawY) < 0.15) rawY = 0;
                if (Math.abs(rawX) < 0.15) rawX = 0;
                if (Math.abs(rawRot) < 0.15) rawRot = 0;

                // 🔑 Critical fix: true zero = Idle
                if (rawY == 0 && rawX == 0 && rawRot == 0) {
                    return new SwerveRequest.Idle();
                }

                return drive
                    .withVelocityX(rawY * MaxSpeed)
                    .withVelocityY(rawX * MaxSpeed)
                    .withRotationalRate(rawRot * MaxAngularRate);
            })
        );

        // Idle while disabled
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

        // Intake
        driverXboxController.rightBumper()
            .debounce(0.25)
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

        SmartDashboard.putNumber("Operator LT", operatorXboxController.getLeftTriggerAxis());
        SmartDashboard.putNumber("Driver Right Trigger", driverXboxController.getRightTriggerAxis());

        // Limelight data
        SmartDashboard.putNumber("Limelight TX", limelight.getTX());
        SmartDashboard.putNumber("Limelight Distance", limelight.getDistanceMeters());
        SmartDashboard.putBoolean("Limelight Has Target", limelight.hasTarget());

        // ✅ Vision-corrected pose
        SmartDashboard.putNumber("Robot X", drivetrain.getPose().getX());
        SmartDashboard.putNumber("Robot Y", drivetrain.getPose().getY());
        SmartDashboard.putNumber("Robot Heading",
            drivetrain.getPose().getRotation().getDegrees());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}