// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DistanceDrive extends CommandBase {
  Drivetrain drivetrain;
  double endpoint;

  /** Creates a new DistanceDrive. */
  public DistanceDrive(Drivetrain drivetrain, double endpoint) {
    this.drivetrain = drivetrain;
    this.endpoint = endpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(-0.3, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    //engage pnuematic breaks here
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0) >= endpoint;
  }
}