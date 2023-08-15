// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cowlib.arm.ArmPose;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class RobotConstants {
        public static final int PCMID = 7;
    }

    public static final class DriveConstants {
        public static final int backLeftMotorID = 6;
        // public static final int backRightMotorID = 11;
        public static final int frontLeftMotorID = 8;
        // public static final int frontRightMotorID = 13;

        public static final boolean currentLimitEnabled = true;
        public static final int currentLimit = 30;

        public static final int shifterID = 4;

        public static final boolean lowGear = false;
    }

    public static final class HatchLatchConstants {
        public static final int latchID = 5;
        public static final int extendID = 6;

        public static final double clapTime = 0.2;
    }

    public static final class CargoIntakeConstants {
        public static final int conveyorMotorID = 2;
    }

    public static final class AutoConstants {
        public static final double taCenter = 0.425; //ta value that is at the center of the charge station

        public static final double chargeStationCenterPose = 4.6; //X position of the center of the charge station
        public static final double outsideCommunityPose = 3; //X position of being outside the community

        public static final double smoothConstant = 15;
    }

    public static final class AutoPIDConstants {
        public static final double kP = 2;
        public static final double kI = 0;
        public static final double kD = 0.01;
    }

    public static class ArmConstants {
        public static final int lowerArmMotorID = 15;
        public static final int upperArmMotorID = 25;
    
        public static final int lowerArmEncoderID = 3;
        public static final int upperArmEncoderID = 4;
    
        public static final ArmPose[] poses = new ArmPose[] {
          // new ArmPose(284.8, 294.8),
          // new ArmPose(318, 254),
          // new ArmPose(2, 220)
          new ArmPose(0, 0),
          new ArmPose(75, -70),
          new ArmPose(100, -100)
        };
    
        public static class PID {
          public static class Upper{
            public static final double kP = 0.2;
            public static final double kI = 0;
            public static final double kD = 0;
          }
          public static class Lower{
            public static final double kP = 0.35;
            public static final double kI = 0;
            public static final double kD = 0.005;
          }
        }
    
        public static class FeedForward {
          public static class Upper{
            public static final double kS = 0;
            public static final double kG = 0;
            public static final double kV = 0;
          }
          public static class Lower{
            public static final double kS = 0;
            public static final double kG = 0;
            public static final double kV = 0;
          }
        }
      }
    
}