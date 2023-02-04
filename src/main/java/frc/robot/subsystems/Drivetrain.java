// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

interface applyConfig {
  public void apply(CANSparkMax motorController);
}

public class Drivetrain extends SubsystemBase {
  private CANSparkMax backLeftMotor = new CANSparkMax(DriveConstants.backLeftMotorID, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(DriveConstants.backRightMotorID, MotorType.kBrushless);
  private CANSparkMax frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);

  private Solenoid shifter = new Solenoid(RobotConstants.PCMID, PneumaticsModuleType.CTREPCM, DriveConstants.shifterID);

  private DifferentialDrive diffDrive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    
    applyConfigs((CANSparkMax mc) -> mc.restoreFactoryDefaults());
    applyConfigs((CANSparkMax mc) -> mc.setSmartCurrentLimit(DriveConstants.currentLimit));

    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);

    diffDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
  }

  void applyConfigs(applyConfig config) {
    config.apply(backLeftMotor);
    config.apply(backRightMotor);
    config.apply(frontLeftMotor);
    config.apply(frontRightMotor);
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
