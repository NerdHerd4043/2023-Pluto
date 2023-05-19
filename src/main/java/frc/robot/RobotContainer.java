// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.auto.BalanceOnPlatform;
import frc.robot.commands.autoCommands.*;
import frc.robot.commands.hatchlatch.Clap;
import frc.robot.subsystems.Arm;
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

  private double kP;
  private double kI;
  private double kD;

  private final Drivetrain drivetrain = new Drivetrain();
  private final CargoIntake cargoIntake = new CargoIntake();
  private final HatchLatch hatchLatch = new HatchLatch();
  private final Arm arm = new Arm();

  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  private PIDController pidController = new PIDController(kP, kI, kD);

  // private static XboxController driveStick = new XboxController(0);
  private static CommandXboxController driveStick = new CommandXboxController(0);
  
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // private final DistanceDrive distanceDrive = new DistanceDrive(drivetrain, 0.3, AutoConstants.taCenter);
  private final DistanceDrive leaveCommunity = new DistanceDrive(drivetrain, -0.6, AutoConstants.outsideCommunityPose);
  private final TimeDrive leaveCommunityTimed = new TimeDrive(drivetrain, -0.6, 4);
  private final PidBalance pidBalance = new PidBalance(
    drivetrain, pidController, gyro,
    () -> Math.abs(limelightTable.getEntry("botpose").getDoubleArray(new Double[0])[0]));

  private final BalanceOnPlatform balanceOnPlatform = new BalanceOnPlatform(drivetrain, pidController, gyro);

  SendableChooser<Command> commandChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // SmartDashboard.putNumber("kP", 0);
    // SmartDashboard.putNumber("kI", 0);
    // SmartDashboard.putNumber("kD", 0);

    // commandChooser.addOption("Balance with Distance", distanceDrive);
    // commandChooser.addOption("Leave the Community", leaveCommunity);
    commandChooser.addOption("Balance with PID", pidBalance);
    commandChooser.addOption("Leave the Community", leaveCommunityTimed);
    commandChooser.addOption("Leave Community and Balance", balanceOnPlatform);

    // SmartDashboard.putData(commandChooser);

    // drivetrain.setDefaultCommand(
    //   new RunCommand(
    //     () -> drivetrain.drive(
    //       driveStick.getLeftY(), 
    //       driveStick.getRightX()
    //     ),
    //     drivetrain
    //   )
    // );

    // arm.setDefaultCommand(
    //   arm.driveMotors(
    //     () -> driveStick.getLeftY(),
    //     () -> driveStick.getRightY()));

    arm.setDefaultCommand(
      arm.adjustCommand(
        () -> driveStick.getRightTriggerAxis() - driveStick.getLeftTriggerAxis()
      ));

    // cargoIntake.setDefaultCommand(
    //   new RunCommand(
    //     () -> cargoIntake.set(
    //       driveStick.getRightTriggerAxis(), 
    //       driveStick.getLeftTriggerAxis()
    //     ), 
    //     cargoIntake)
    // );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveStick.b().onTrue(new InstantCommand(arm::nextPose, arm));
    driveStick.a().onTrue(new InstantCommand(arm::previousPose, arm));
    driveStick.y().onTrue(new InstantCommand(arm::printEncoders, arm));


    // new JoystickButton(driveStick, Button.kX.value).onTrue(new InstantCommand(hatchLatch::toggleLatch, hatchLatch));
    // new JoystickButton(driveStick, Button.kY.value).onTrue(new InstantCommand(hatchLatch::toggleExtend, hatchLatch));
    // new JoystickButton(driveStick, Button.kA.value).toggleOnTrue(new RunCommand(() -> cargoIntake.in(), cargoIntake));
    // new JoystickButton(driveStick, Button.kB.value).toggleOnTrue(new RunCommand(() -> cargoIntake.out(), cargoIntake));
    // new JoystickButton(driveStick, Button.kLeftBumper.value).onTrue(new InstantCommand(drivetrain::shiftUp, drivetrain));
    // new JoystickButton(driveStick, Button.kRightBumper.value).onTrue(new InstantCommand(drivetrain::shiftDown, drivetrain));
    // new JoystickButton(driveStick, Button.kBack.value).toggleOnTrue(new Clap(hatchLatch));

    // new JoystickButton(driveStick, Button.kStart.value).onTrue(getBreakCommand());
    // new JoystickButton(driveStick, Button.kBack.value).onTrue(getCoastCommand());
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

  public void enableArm() {
    arm.enable();
  }

  public void disableArm() {
    arm.disable();
  }

  public void updatePIDValues() {
    pidController.setP(SmartDashboard.getNumber("kP", 0));
    pidController.setI(SmartDashboard.getNumber("kI", 0));
    pidController.setD(SmartDashboard.getNumber("kD", 0));
  }

  public Command getCoastCommand(){
    return new InstantCommand(drivetrain::setCoastMode);
  }

  public Command getBreakCommand(){
    return new InstantCommand(drivetrain::setBreakMode);
  }

}