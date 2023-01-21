// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HatchLatchConstants;
import frc.robot.Constants.RobotConstants;

/**
 * Iain hasn definitely been here
 */
public class HatchLatch extends SubsystemBase {
  Solenoid latch = new Solenoid(RobotConstants.PCMID, PneumaticsModuleType.CTREPCM, HatchLatchConstants.latchID);
  Solenoid extend = new Solenoid(RobotConstants.PCMID, PneumaticsModuleType.CTREPCM, HatchLatchConstants.extendID);

  /** Creates a new HatchLatch. */
  public HatchLatch() {}

  public void toggleLatch() {
    setLatch(!latch.get());
  }

  public void setLatch(boolean a) {
    latch.set(a);
  }

  public void toggleExtend() {
    setExtend(!extend.get());
  }

  public void setExtend(boolean a) {
    extend.set(a);
  }

  public void setHatchLatch(boolean b_latch, boolean b_extend) {
    setLatch(b_latch);
    setExtend(b_extend);
  }

  public void tuck() {
    setLatch(false);
    setExtend(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
