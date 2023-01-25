// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

public class Drivetrain extends SubsystemBase {
  WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(DriveConstants.backLeftMotorID);
  WPI_TalonSRX backRightMotor = new WPI_TalonSRX(DriveConstants.backRightMotorID);
  WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(DriveConstants.frontLeftMotorID);
  WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(DriveConstants.frontRightMotorID);

  Solenoid shifter = new Solenoid(RobotConstants.PCMID, PneumaticsModuleType.CTREPCM, DriveConstants.shifterID);

  DifferentialDrive diffDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    backLeftMotor.setSafetyEnabled(false);
    backRightMotor.setSafetyEnabled(false);
    frontLeftMotor.setSafetyEnabled(false);
    frontRightMotor.setSafetyEnabled(false);

    backLeftMotor.enableCurrentLimit(DriveConstants.currentLimitEnabled);
    backRightMotor.enableCurrentLimit(DriveConstants.currentLimitEnabled);
    frontLeftMotor.enableCurrentLimit(DriveConstants.currentLimitEnabled);
    frontRightMotor.enableCurrentLimit(DriveConstants.currentLimitEnabled);
    
    backLeftMotor.configContinuousCurrentLimit(DriveConstants.currentLimit);
    backRightMotor.configContinuousCurrentLimit(DriveConstants.currentLimit);
    frontLeftMotor.configContinuousCurrentLimit(DriveConstants.currentLimit);
    frontRightMotor.configContinuousCurrentLimit(DriveConstants.currentLimit);

    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    diffDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
  }

  public void drive(double fwd, double rot) {
    drive(fwd, rot, true);
  }

  public void drive(double fwd, double rot, boolean squared_inputs) {
    diffDrive.arcadeDrive(fwd, rot, squared_inputs);
  }

  public void stop(){
    drive(0, 0);
  }

  public void shift(boolean a) {
    shifter.set(a);
  }

  public void shiftUp() {
    shift(!DriveConstants.lowGear);
  }

  public void shiftDown() {
    shift(DriveConstants.lowGear);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
