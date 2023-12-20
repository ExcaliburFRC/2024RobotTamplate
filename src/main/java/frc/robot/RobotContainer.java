// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static frc.lib.Color.Colors.WHITE;
import static frc.robot.subsystems.LEDs.LEDPattern.SOLID;

public class RobotContainer {
  // subsystems
  private final Swerve swerve = new Swerve();
  private final Superstructure superstructure = new Superstructure();
  private final LEDs leds = LEDs.getInstance();

  // controllers
  private final CommandPS4Controller driver = new CommandPS4Controller(0);
  private final CommandPS4Controller operator = new CommandPS4Controller(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
            swerve.driveSwerveCommand(
                    ()-> applyDeadband(-driver.getLeftY(), 0.05),
                    ()-> applyDeadband(-driver.getLeftX(), 0.05),
                    ()-> applyDeadband(-driver.getRightX(), 0.05),
                    driver.L2().negate(),
                    driver::getR2Axis,
                    ()-> getAngleFromButtons(driver.triangle(), driver.circle(), driver.cross(), driver.square()))
            );

    driver.PS().onTrue(swerve.resetOdometryAngleCommand());
    driver.touchpad().whileTrue(toggleMotorsIdleMode().alongWith(leds.applyPatternCommand(SOLID, WHITE.color)));

    operator.triangle().toggleOnTrue(superstructure.placeOnHighCommand(driver.R1()));
    operator.circle().toggleOnTrue(superstructure.placeOnMidCommand(driver.R1()));
    operator.cross().toggleOnTrue(superstructure.placeOnLowCommand());

    operator.square().onTrue(superstructure.lockArmCommand());
    operator.R2().onTrue(superstructure.arm.forceLockArm());
    operator.R1().toggleOnTrue(superstructure.intakeFromShelfCommand());
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
