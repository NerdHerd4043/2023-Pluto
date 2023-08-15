// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.PID;

public class LonelyPID extends PIDSubsystem {

  private CANSparkMax lowerArmMotor = new CANSparkMax(ArmConstants.lowerArmMotorID, MotorType.kBrushless);
  private WPI_CANCoder lowerArmEncoder = new WPI_CANCoder(ArmConstants.lowerArmEncoderID);

  /** Creates a new LonelyPID. */
  public LonelyPID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(
          PID.Lower.kP,
          PID.Lower.kI,
          PID.Lower.kD));

        lowerArmMotor.restoreFactoryDefaults();
        lowerArmMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    SmartDashboard.putNumber("Encoder Value", getEncoder());
    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Setpoint", setpoint);
    lowerArmMotor.setVoltage(-output);
  }

  public double getEncoder(){
    return lowerArmEncoder.getAbsolutePosition();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoder();
  }
}
