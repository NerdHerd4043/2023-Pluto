// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.PID;
import frc.robot.Constants.ArmConstants.FeedForward.Upper;
import frc.robot.Constants.ArmConstants.FeedForward;;

public class LonelyProfPID extends ProfiledPIDSubsystem {

  private CANSparkMax upperArmMotor = new CANSparkMax(ArmConstants.upperArmMotorID, MotorType.kBrushless);
  private WPI_CANCoder upperArmEncoder = new WPI_CANCoder(ArmConstants.upperArmEncoderID);
  private ArmFeedforward feedforward = new ArmFeedforward(Upper.kS, Upper.kG, Upper.kV);

  /** Creates a new LonelyProfPID. */
  public LonelyProfPID() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
          PID.Upper.kP,
          PID.Upper.kI,
          PID.Upper.kD,
          new TrapezoidProfile.Constraints(50 , 50)));
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double ff = feedforward.calculate(setpoint.position, setpoint.velocity);
    // SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Setpoint", setpoint.position);
    // SmartDashboard.putNumber("Fed Firward", feedforward.calculate(setpoint.position, setpoint.velocity));
    SmartDashboard.putNumberArray("PID, FF, and Combo", new double[]{output, ff, output + ff});
    // SmartDashboard.putNumber("Setpoint Velocity", setpoint.velocity);
    upperArmMotor.setVoltage(output + ff);  
    // upperArmMotor.setVoltage(feedforward.calculate(setpoint.position, setpoint.velocity));
  }

  public double getEncoder(){
    return upperArmEncoder.getAbsolutePosition();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoder();
  }
  
  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Encoder Value", getEncoder());
  }
}
