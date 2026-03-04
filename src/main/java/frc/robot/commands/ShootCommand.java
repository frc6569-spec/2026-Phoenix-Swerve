package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;

public class ShootCommand extends SequentialCommandGroup {

    // Easy to tune delay
    private static final double SHOOTER_STABILIZE_DELAY = 0.3;

    public ShootCommand(Shooter shooter, Feeder feeder) {

        addCommands(

            // Start shooter
            shooter.runOnce(shooter::spinShooter),

            // Wait until shooter reaches speed
            new WaitUntilCommand(shooter::atSpeed),

            // Stabilize flywheel
            new WaitCommand(SHOOTER_STABILIZE_DELAY),

            // Run feeder
            feeder.run(() -> feeder.runFeeder())

        );

        addRequirements(shooter, feeder);

        // This runs when trigger is released
        finallyDo((interrupted) -> {
            shooter.stopShooter();
            feeder.stopFeeder();
        });
    }
}