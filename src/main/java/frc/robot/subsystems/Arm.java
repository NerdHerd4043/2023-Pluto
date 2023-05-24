// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import cowlib.DualProfiledPIDSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.PID;
import static frc.robot.Constants.ArmConstants.*;

public class Arm extends DualProfiledPIDSubsystem {

  ShuffleboardTab shuffTab = Shuffleboard.getTab("Arm");

  private CANSparkMax lowerArmMotor = new CANSparkMax(ArmConstants.lowerArmMotorID, MotorType.kBrushless);
  private CANSparkMax upperArmMotor = new CANSparkMax(ArmConstants.upperArmMotorID, MotorType.kBrushless);

  private WPI_CANCoder lowerArmEncoder = new WPI_CANCoder(ArmConstants.lowerArmEncoderID);
  private WPI_CANCoder upperArmEncoder = new WPI_CANCoder(ArmConstants.upperArmEncoderID);

  private NetworkTableEntry lowerArmEncoderEntry;
  private NetworkTableEntry upperArmEncoderEntry;

  private double pose = 0;

  private ArmFeedforward lowerArmFeedForward = new ArmFeedforward(FeedForward.Lower.kS, FeedForward.Lower.kG, FeedForward.Lower.kV);
  private ArmFeedforward upperArmFeedForward = new ArmFeedforward(FeedForward.Upper.kS, FeedForward.Upper.kG, FeedForward.Upper.kV);

  /** Creates a new Arm. */
  public Arm() {
    super(
        //PID Controller A for lower arm
        new ProfiledPIDController(
          PID.Upper.kP,
          PID.Upper.kI,
          PID.Upper.kD,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(0.2, 0.2)),
        
        //PID Controller B for upper arm
        new ProfiledPIDController(
          PID.Lower.kP,
          PID.Lower.kI,
          PID.Lower.kD,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(0.2, 0.2))); 

    lowerArmMotor.restoreFactoryDefaults();
    upperArmMotor.restoreFactoryDefaults();

    lowerArmMotor.setIdleMode(IdleMode.kBrake);
    upperArmMotor.setIdleMode(IdleMode.kBrake);

    // PID Controllers in the Arm Tab
    ShuffleboardLayout pidContainer = shuffTab
      .getLayout("PID Controllers", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withPosition(0, 0);

    pidContainer.add("Lower PID Controller", m_controllerA);
    pidContainer.add("Upper PID Controller", m_controllerB);

    // Encoders in the Arm Tab
    ShuffleboardLayout encoderContainer = shuffTab
      .getLayout("Encoders", BuiltInLayouts.kList)
      .withSize(1, 2)
      .withPosition(2, 0);

    encoderContainer.add("Lower Arm Encoder", lowerArmEncoder);
    encoderContainer.add("Upper Arm Encoder", upperArmEncoder);
  }

  public double clamp(double value, double min, double max){
    return Math.max(min, Math.min(max, value));
  }

  public void nextPose() {
    changePose(1);
  }

  public void previousPose() {
    changePose(-1);
  }

  public void changePose(int nextPose) {
    pose = clamp(pose + nextPose, 0, poses.length - 1);
    updateGoals();
  }

  public void adjustPosition(double input) {
    pose = clamp(pose + input, 0, poses.length - 1);
    updateGoals();
  }

  public CommandBase adjustCommand(DoubleSupplier input) {
    return this.run(() -> this.adjustPosition(input.getAsDouble() / 50));
  }

  public CommandBase driveLowerMotor(DoubleSupplier speed){
    return this.run(() -> lowerArmMotor.set(speed.getAsDouble() * 0.3));
  }

  public CommandBase driveUpperMotor(DoubleSupplier speed){
    return this.run(() -> upperArmMotor.set(speed.getAsDouble() * 0.3));
  }

  private double lerp(double a, double b, double f) {
    return (a * (1.0 - f)) + (b * f);
  }

  public void updateGoals() {
    int lowerBound = Math.min((int) Math.floor(pose), poses.length - 2);
    int upperBound = Math.min((int) Math.ceil(pose), poses.length - 1);

    double fraction = pose - lowerBound;
    // setGoals(poses[(int)pose].lower(), poses[(int)pose].upper());

      double upperGoal = lerp(
        poses[lowerBound].upper(), 
        poses[upperBound].upper(), 
        fraction);

      double lowerGoal = lerp(
        poses[lowerBound].lower(), 
        poses[upperBound].lower(), 
        fraction);

    setGoals(lowerGoal,upperGoal);
  }

  public void printEncoders() {
    SmartDashboard.putString("Encoder Values", 
    "Lower: " + lowerArmEncoder.getAbsolutePosition() +
    ", Upper: " + upperArmEncoder.getAbsolutePosition());
  }

  public CommandBase driveMotors(DoubleSupplier speed, DoubleSupplier speed2){
    return this.run(() -> {
      lowerArmMotor.set(speed.getAsDouble() * 0.4);
      upperArmMotor.set(speed2.getAsDouble() * 0.2);
    });
  } 

  @Override
  public void useOutput(double outputLower, double outputUpper, State setpointLower, State setpointUpper) {
    SmartDashboard.putNumber("Lower Output", outputLower);
    SmartDashboard.putNumber("Upper Output", outputUpper);

    // PID driving motor
    // lowerArmMotor.setVoltage(outputLower);
    // upperArmMotor.setVoltage(outputUpper);

    // feedforward driving motor??
    // lowerArmMotor.setVoltage(lowerArmFeedForward.calculate(setpointLower.position, setpointLower.velocity));
    // upperArmMotor.setVoltage(upperArmFeedForward.calculate(setpointUpper.position, setpointUpper.velocity));

    // both driving motor???????
    // lowerArmMotor.setVoltage(outputLower + lowerArmFeedForward.calculate(setpointLower.position, setpointLower.velocity));
    // upperArmMotor.setVoltage(outputUpper + upperArmFeedForward.calculate(setpointUpper.position, setpointUpper.velocity));

  }

  @Override
  public double getMeasurement(Controller controller) {
    // Return the process variable measurement here
    switch(controller){
      case A: return lowerArmEncoder.getAbsolutePosition();
      case B: return upperArmEncoder.getAbsolutePosition();

      default: return 0;
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    double[] goals = this.getGoals();
    SmartDashboard.putNumber("Lower Goal", goals[0]);
    SmartDashboard.putNumber("Upper Goal", goals[1]);
    // System.out.println(
      // "Lower: " + lowerArmEncoder.getAbsolutePosition() +
      // ", Upper: " + upperArmEncoder.getAbsolutePosition());
  }
}