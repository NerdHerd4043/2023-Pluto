// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private CANSparkMax lowerArmMotor = new CANSparkMax(ArmConstants.lowerArmMotorID, MotorType.kBrushless);
  private CANSparkMax upperArmMotor = new CANSparkMax(ArmConstants.upperArmMotorID, MotorType.kBrushless);

  /** Creates a new Arm. */
  public Arm() {
    lowerArmMotor.restoreFactoryDefaults();
    upperArmMotor.restoreFactoryDefaults();

    lowerArmMotor.setIdleMode(IdleMode.kBrake);
    upperArmMotor.setIdleMode(IdleMode.kBrake);
  }

  public CommandBase driveLowerMotor(DoubleSupplier speed){
    return this.run(() -> lowerArmMotor.set(speed.getAsDouble() * 0.3));
  }

  public CommandBase driveUpperMotor(DoubleSupplier speed){
    return this.run(() -> upperArmMotor.set(speed.getAsDouble() * 0.3));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
