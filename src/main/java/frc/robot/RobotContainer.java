// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.swerve.Swerve;

import static frc.lib.Color.Colors.WHITE;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class RobotContainer {
  // subsystems
  private final Swerve swerve = new Swerve();
  private final LEDs leds = LEDs.getInstance();

  // controllers
  private final CommandPS5Controller controller = new CommandPS5Controller(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
            swerve.driveSwerveCommand(
                    ()-> - controller.getLeftY(),
                    ()-> -controller.getLeftX(),
                    ()-> -controller.getRightX(),
                    controller.L2(),
                    controller::getR2Axis,
                    ()-> getAngleFromButtons(controller.triangle(), controller.circle(), controller.cross(), controller.square()))
            );

    controller.touchpad().whileTrue(toggleMotorsIdleMode().alongWith(leds.applyPatternCommand(SOLID, WHITE.color)));
  }

  public double getAngleFromButtons(Trigger triangle, Trigger circle, Trigger cross, Trigger square){
    if (triangle.getAsBoolean()) return 0;
    if (circle.getAsBoolean()) return 90;
    if (cross.getAsBoolean()) return 180;
    if (square.getAsBoolean()) return 270;
    return -1;
  }

  public Command toggleMotorsIdleMode() {
    return new ParallelCommandGroup(
            swerve.toggleIdleModeCommand()
            // add other subsystems here
    );
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
