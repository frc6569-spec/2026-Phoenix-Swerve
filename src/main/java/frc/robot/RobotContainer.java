package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

    //----------------------------
    // Subsystems
    //----------------------------
    private final Feeder feeder = new Feeder();
    private final Intake intake = new Intake(feeder);
    private final Shooter shooter = new Shooter(feeder);
    private final Limelight limelight = new Limelight();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //----------------------------
    // PathPlanner Auto Chooser
    //----------------------------
    private final SendableChooser<Command> autoChooser;

    //----------------------------
    // Drive config
    //----------------------------
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //----------------------------
    // Controllers
    //----------------------------
    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);

    public RobotContainer() {

        configureBindings();

        //--------------------------------
        // Build PathPlanner auto chooser
        //--------------------------------
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
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

                //--------------------------------
                // LIMELIGHT AUTO AIM
                //--------------------------------
                double rot;

                boolean autoAim =
                    driverXboxController.getLeftTriggerAxis() > 0.5;

                if (autoAim && limelight.hasTarget()) {

                    double tx = limelight.getTX();

                    double kP = 0.02;

                    if (Math.abs(tx) < 1.0)
                        rot = 0;
                    else
                        rot = tx * kP;

                    double distance = limelight.getDistanceMeters();
                    double speed = shooter.getSpeedForDistance(distance);

                    shooter.setSpeed(speed);

                } else {

                    rot = squaredRot * MaxAngularRate;

                }

                return drive
                    .withVelocityX(squaredY * MaxSpeed * speedMultiplier)
                    .withVelocityY(squaredX * MaxSpeed * speedMultiplier)
                    .withRotationalRate(rot * speedMultiplier);
            })
        );

        //----------------------------
        // Idle while disabled
        //----------------------------
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

        //----------------------------
        // SysId routines
        //----------------------------
        driverXboxController.back().and(driverXboxController.y())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        driverXboxController.back().and(driverXboxController.x())
            .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        driverXboxController.start().and(driverXboxController.y())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        driverXboxController.start().and(driverXboxController.x())
            .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //----------------------------
        // Reset heading
        //----------------------------
        driverXboxController.back().and(driverXboxController.start())
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //----------------------------
        // Shooter test spin
        //----------------------------
        driverXboxController.y()
            .whileTrue(
                shooter.runEnd(
                    shooter::spinShooter,
                    shooter::stopShooter
                )
            );

        //----------------------------
        // Shooter + Feeder
        //----------------------------
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

        //----------------------------
        // Intake Controls
        //----------------------------
        driverXboxController.rightBumper()
            .debounce(0.25)
            .onTrue(intake.runOnce(intake::toggleExtendPushback));

        driverXboxController.leftBumper()
            .debounce(0.25)
            .onTrue(intake.runOnce(intake::retract));

        //----------------------------
        // Operator Feeder Test
        //----------------------------
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

        SmartDashboard.putNumber(
            "Driver Right Trigger",
            driverXboxController.getRightTriggerAxis()
        );

        SmartDashboard.putNumber("Limelight TX", limelight.getTX());
        SmartDashboard.putNumber("Limelight Distance", limelight.getDistanceMeters());
        SmartDashboard.putBoolean("Limelight Has Target", limelight.hasTarget());
    }

    //--------------------------------
    // Autonomous
    //--------------------------------
    public Command getAutonomousCommand() {

        return autoChooser.getSelected();
    }
}