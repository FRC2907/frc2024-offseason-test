package frc.robot.constants;

import frc.robot.subsystems.Drivetrain.DriveMode;

public class Control {
    
    public static class arm { //TODO arm control constants
        public static final double ENCODER_POS_UNIT_PER_DEGREE = 0; // tick/deg
        public static final double ENCODER_VEL_UNIT_PER_DEGREE_PER_SECOND = 0; //tick/deg/s

        public static final double kP = 1;
        public static final double kD = 1;
        public static final double kStartPosition = 0;
        public static final double kAmpPosition = 90;
        public static final double kAmpShootPosition = 95;
        public static final double kSpeakerPosition = 30;
        public static final double kIntakePosition = 30;
        public static final double kHoldingPosition = 50;
        public static final double kNeutralPosition = 30;
        public static final double kMinPosition = 0;
        public static final double kMaxPosition = 100;

        public static final double kManualControlDiff = 2;
        public static final double kPositionHysteresis = 2;
        public static final double kVelocityHysteresis = 2;
    }

    public static class drivetrain {
        public static final DriveMode kDefaultDriveMode = DriveMode.LOCAL_FORWARD;
    }

    public static class intake { //TODO intake control constants
        public static final double ENCODER_VEL_UNIT_PER_INTAKE_MPS = 0;

        public static final double kP = 1;
        public static final double kD = 1;

        public static final double kIntakingSpeed = 60; // m/s
        public static final double kOff = 0;
        public static final double kOutakingSpeed = -60;
        public static final double kVelocityHysteresis = 2;
        public static final double kOnHysteresis = 3;
        public static final double kAverageCurrent = 0;
        public static final double kCurrentHystereis = 2;

        public static final int kArrayLength = 50;
    }

    public static class shooter { //TODO shooter control constants
        public static final double ENCODER_VEL_UNIT_PER_SHOOTER_MPS = 0;

        public static final double kP = 1;
        public static final double kD = 1;

        public static final double kSpeakerSpeed = 100; // m/s
        public static final double kAmpSpeed = 30;
        public static final double kOff = 0;
        public static final double kVelocityHysteresis = 2;
    }
}