// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PidAuto extends PIDCommand {
  
  private final Drivetrain drivetrain;

  /** Creates a new PidAuto. */
  public PidAuto(Drivetrain drivetrain, DoubleSupplier xPose, DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD) {
    super(
        // The controller that the command will use
        // new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD),
        new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble()),
        // This should return the measurement
        xPose,
        // This should return the setpoint (can also be a constant)
        () -> AutoConstants.chargeStationCenter,
        // This uses the output
        output -> {
          // Use the output here
          if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1){
            drivetrain.drive(-output, 0);
          }
          SmartDashboard.putNumber("PID Output", output);
        },
        drivetrain);

    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
