// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.RobotConstants;

public class PidDriveSubsystem extends PIDSubsystem {

  private final Drivetrain drivetrain;

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

  /** Creates a new PIDSubsystem. */
  public PidDriveSubsystem(Drivetrain drivetrain) {
    super(
        // The PIDController used by the subsystem
        new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD));

    this.drivetrain = drivetrain;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    drivetrain.drive(output, 0);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}