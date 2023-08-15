// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armyArm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LonelyPID;
import frc.robot.subsystems.LonelyProfPID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LonelyProfForward extends InstantCommand {
  LonelyProfPID pid;

  public LonelyProfForward(LonelyProfPID pid) {
    this.pid = pid;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.pid);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setGoal(100);
    pid.enable();
  }
}