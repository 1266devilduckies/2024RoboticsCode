package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

    public static class OperatorConstants {
        public static final int port = 1;
        public static final int movementAxis = 5;
        public static final double movementDeadband = 0.5;
        public static final int elbowMovementAxis = 1;
    }
    
    public static class DriverConstants {
        public static final int port = 0;
        public static final double deadbandLeftJoystick = 0.05;
        public static final double deadbandRightJoystick = deadbandLeftJoystick;

        public static final int forwardAxis = 1;
        public static final int strafeAxis = 0;
        public static final int turnAxis = 2;

        public static final double autoMaxSpeedMeters = 5;
        public static final double autoMaxAccelerationMeters = 3;

        public static final double speedMultiplier = 5; //not currently used
        public static final double turnSpeedMultiplier = 5; //not currently used
        public static final double speedMetersPerSecond = Units.feetToMeters(4) * speedMultiplier;
        public static double speedRadiansPerSecond = 0.5*Math.PI * turnSpeedMultiplier;
    }

    public static class Intake {
        public static final int spinID = 24;
        public static final int spinIDFollower = 25; // shared with arm Encoder

        // Speeds
        public static final double intakeStoppedSpeed = 0;
        public static final double intakeTakeInSpeed = 1;
        public static final double intakeShootOutSpeed = -1;

        // PID
        public static final double intakeP = 0;
        public static final double intakeI = 0;
        public static final double intakeD = 0;
    }

    public static class Arm {
        public static final int armID = 6;
        public static final int armFollowerID = 23;
        public static final int encoderID = 25; //shared with spin follower

        // IGNORE THIS, AVOID CHANGING
        public static final double armDefaultP = 0;
        public static final double armDefaultI = 0;
        public static final double armDefaultD = 0;

        // ARM FEED FORWARD (Don't need)
        public static final double armS = 4; // voltage required to start movement
        public static final double armG = 1; // voltage required to hold position
        public static final double armV = 6; // speed

        // PID
        public static final double armP = 0.5;
    }

    public static class Launch {
        public static final int launchMotorID = 7;
        public static final int launchMotorFollowerID = 8;

        // PID
        public static final double launchP = 0;
        public static final double launchI = 0;
        public static final double launchD = 0;
    }

    public static class AutoTrajectoryFileNames{
        public static final String TEST_STRAIGHT = "Test Straight Path";
        public static final String TEST_CURVE = "Test Curve Path";
        public static final String TWO_NOTE = "2 Note Auto";
        public static final String ONE_MORE_NOTE = "1 More Note Auto";
    }
    
}
