// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class SelfAdjust extends CommandBase {
  Drivetrain drivetrain;
  double speed;
  double timerStart;
  double timerEnd;
  double[] emptyArray = {}; 

  /** Creates a new DistanceDrive. */
  public SelfAdjust(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    speed = 0;
    timerStart = 0;
    timerEnd = 2;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStart = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(SmartDashboard.getNumber("X Position", 0) < AutoConstants.XFarCenter - 0.07){
          speed = -0.3;
    }
    else if(SmartDashboard.getNumber("X Position", 0) < AutoConstants.XFarCenter){
      speed = -0.2;
    }
    else if(SmartDashboard.getNumber("X Position", 0) > AutoConstants.XCloseCenter){
      speed = 0.2;
    }
    else{
      speed = 0;
      timerStart = Timer.getFPGATimestamp();
    }

    // if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(emptyArray)[0] < 4.45){
    //   speed = -0.4;
    // }
    // else if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(emptyArray)[0] >= 4.45 && 
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(emptyArray)[0] <= 4.5){
    //   speed = -0.15;
    // }
    // else{
    //   speed = 0;
    // }

    drivetrain.drive(speed, 0);
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
    return false;
    // return ((Timer.getFPGATimestamp() - timerStart) > timerEnd) && (speed == 0);
  }
}