// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.hatchlatch.Clap;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HatchLatch;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final CargoIntake cargoIntake = new CargoIntake();
  private final HatchLatch hatchLatch = new HatchLatch();

  private static XboxController driveStick = new XboxController(0);

  private final DistanceDrive distanceDrive = new DistanceDrive(drivetrain, AutoConstants.taCenter);
  private final SelfAdjust selfAdjust = new SelfAdjust(drivetrain);

  SendableChooser<Command> commandChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    commandChooser.addOption("Balance with Distance", distanceDrive);
    commandChooser.addOption("Self Adjust on Charge Station", selfAdjust);

    SmartDashboard.putData(commandChooser);

    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> drivetrain.drive(
          driveStick.getLeftY(), 
          driveStick.getRightX()
        ),
        drivetrain
      )
    );

    cargoIntake.setDefaultCommand(
      new RunCommand(
        () -> cargoIntake.set(
          driveStick.getRightTriggerAxis(), 
          driveStick.getLeftTriggerAxis()
        ), 
        cargoIntake)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driveStick, Button.kX.value).onTrue(new InstantCommand(hatchLatch::toggleLatch, hatchLatch));
    new JoystickButton(driveStick, Button.kY.value).onTrue(new InstantCommand(hatchLatch::toggleExtend, hatchLatch));
    new JoystickButton(driveStick, Button.kA.value).toggleOnTrue(new RunCommand(() -> cargoIntake.in(), cargoIntake));
    new JoystickButton(driveStick, Button.kB.value).toggleOnTrue(new RunCommand(() -> cargoIntake.out(), cargoIntake));
    new JoystickButton(driveStick, Button.kLeftBumper.value).onTrue(new InstantCommand(drivetrain::shiftUp, drivetrain));
    new JoystickButton(driveStick, Button.kRightBumper.value).onTrue(new InstantCommand(drivetrain::shiftDown, drivetrain));
    new JoystickButton(driveStick, Button.kBack.value).toggleOnTrue(new Clap(hatchLatch));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return commandChooser.getSelected();
  }
}
