// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.lib.Color.Colors.WHITE;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class RobotContainer {
    // subsystems
    private final Swerve swerve = new Swerve();
    private final LEDs leds = LEDs.getInstance();

    // controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(0);

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("stuff", swerve.doABunchOfCoolStuffWithTheRobotCommand());
    }

    private void configureBindings() {
        swerve.setDefaultCommand(
                swerve.driveSwerveCommand(
                        () -> applyDeadband(-controller.getLeftY(), 0.07),
                        () -> applyDeadband(-controller.getLeftX(), 0.07),
                        () -> applyDeadband(-controller.getRightX(), 0.07),
                        controller.L2().negate(),
                        controller::getR2Axis,
                        () -> -1));

        controller.touchpad().whileTrue(toggleMotorsIdleMode().alongWith(leds.applyPatternCommand(SOLID, WHITE.color)));

//        controller.PS().onTrue(swerve.resetOdometryAngleCommand());
        controller.PS().onTrue(swerve.setOdometryPositionCommand(new Pose2d(0, 0, new Rotation2d(0))));

        controller.cross().onTrue(swerve.pidToPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))));
    }

    public double getAngleFromButtons(Trigger triangle, Trigger circle, Trigger cross, Trigger square) {
        if (triangle.getAsBoolean()) return 0;
        if (circle.getAsBoolean()) return 90;
        if (cross.getAsBoolean()) return 180;
        if (square.getAsBoolean()) return 270;
        return -1;
    }
    // getAngleFromButtons(controller.triangle(), controller.circle(), controller.cross(), controller.square()))

    public Command toggleMotorsIdleMode() {
        return new ParallelCommandGroup(
                swerve.toggleIdleModeCommand()
                // add other subsystems here
        );
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Auto1");
    }
}
