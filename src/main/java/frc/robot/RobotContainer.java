// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Slide;
import frc.robot.subsystems.Climber;
import frc.robot.commands.ShootCommand;

public class RobotContainer {
    //Subsystems
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Feeder feeder = new Feeder();
    private final Slide slide = new Slide();
    private final Climber climber = new Climber();
    private boolean climbEnabled = false;
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXboxController = new CommandXboxController(0);
    private final CommandXboxController operatorXboxController = new CommandXboxController(1);    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
            // 1. Get raw inputs
            double rawY = -driverXboxController.getLeftY();
            double rawX = -driverXboxController.getLeftX();
            double rawRot = -driverXboxController.getRightX();

            // 2. Square the inputs (Controller Squaring)
            // This creates the "slow zone" in the middle of the stick
            double squaredY = Math.copySign(rawY * rawY, rawY);
            double squaredX = Math.copySign(rawX * rawX, rawX);
            double squaredRot = Math.copySign(rawRot * rawRot, rawRot);

            // 3. Dynamic Speed Multiplier (Slow Mode)
            // If Left Trigger is held, go 50% speed. Otherwise, 100%.
            double speedMultiplier = driverXboxController.getLeftTriggerAxis() > 0.5 ? 0.5 : 1.0;

            return drive
                .withVelocityX(squaredY * MaxSpeed * speedMultiplier)
                .withVelocityY(squaredX * MaxSpeed * speedMultiplier)
                .withRotationalRate(squaredRot * MaxAngularRate * speedMultiplier);
        })
    );

    // --- Keep your existing triggers below ---
    
    // Idle while disabled
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    driverXboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverXboxController.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverXboxController.getLeftY(), -driverXboxController.getLeftX()))));

    // SysId routines
    driverXboxController.back().and(driverXboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverXboxController.back().and(driverXboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverXboxController.start().and(driverXboxController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverXboxController.start().and(driverXboxController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset heading (with start and back button)
    driverXboxController.back().and(driverXboxController.start())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
    
    // Shooter controls
    driverXboxController.rightTrigger(.5)
        .whileTrue(new ShootCommand(shooter, feeder));

    // ----------------------------
    // Intake Controls
    // ----------------------------
    // Extend intake and start rollers automatically
    driverXboxController.rightBumper()
        .debounce(0.25)    
        .and(() -> !climbEnabled)
        .onTrue(
            intake.runOnce(intake::extendIntake)
                .andThen(new WaitUntilCommand(intake::isExtended))
                .andThen(intake.runOnce(intake::runIntake))
        );
    // Retract intake and stop rollers
    driverXboxController.leftBumper()
        .debounce(0.25)
        .onTrue(
            intake.runOnce(() -> {
                intake.stopIntake();
                intake.retractIntake();
            })
        );

    // ----------------------------    
    // Operator controls
    // ----------------------------
    // Extend Slide
    operatorXboxController.a().onTrue(slide.runOnce(slide::extend));
    operatorXboxController.b().onTrue(slide.runOnce(slide::retract));

    // Enable Climber
    operatorXboxController.start().onTrue(
        new InstantCommand(() -> climbEnabled = !climbEnabled)
    );
    // climb up
    operatorXboxController.rightTrigger(0.1)
    .and(() -> climbEnabled)
    .whileTrue(
        climber.runEnd(
            () -> climber.climbUp(),   // while held
            () -> climber.stop()       // when released
        )
    );

    // climb down
    operatorXboxController.leftTrigger(0.1)
    .and(() -> climbEnabled)
    .whileTrue(
        climber.runEnd(
            () -> climber.climbDown(),
            () -> climber.stop()
        )
    );

    }

    public void periodic() {
        SmartDashboard.putBoolean("Climb Enabled", climbEnabled);
        SmartDashboard.putNumber(
            "Operator RT",
            operatorXboxController.getRightTriggerAxis()
        );

        SmartDashboard.putNumber(
            "Operator LT",
            operatorXboxController.getLeftTriggerAxis()
        );
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }



}
