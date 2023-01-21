// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hatchlatch;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HatchLatchConstants;
import frc.robot.subsystems.HatchLatch;

public class Clap extends CommandBase {
  HatchLatch hatchLatch;

  double lastSwitch = 0;

  /** Creates a new Clap. */
  public Clap(HatchLatch hatchLatch) {
    this.hatchLatch = hatchLatch;

    addRequirements(this.hatchLatch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Spam Started");
    lastSwitch = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();

    if (currentTime >= lastSwitch + HatchLatchConstants.clapTime) {
      lastSwitch = currentTime;

      hatchLatch.toggleLatch();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Spam Stopped");
    hatchLatch.tuck();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
