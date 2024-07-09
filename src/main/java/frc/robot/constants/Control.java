package frc.robot.constants;

public class Control {
    
    public static class arm { //TODO arm control constants
        public static final double TICK_PER_DEGREE = 0;
        public static final double kP = 1;
        public static final double kD = 1;
        public static final double kStartPosition = 0;
        public static final double kAmpPosition = 90;
        public static final double kAmpShootPosition = 95;
        public static final double kSpeakerPosition = 30;
        public static final double kIntakePosition = 30;
        public static final double kManualControlDiff = 2;
        public static final double kPositionHysteresis = 2;
        public static final double kMinPosition = 0;
        public static final double kMaxPosition = 100;
        public static final double kVelocityHysteresis = 2;
    }

    public static class drivetrain {}

    public static class intake { //TODO intake control constants
        public static final double ENCODER_RPM_PER_WHEEL_RPM = 0;
        public static final double kP = 1;
        public static final double kD = 1;
        public static final double kIntakingRPM = 60;
        public static final double kOff = 0;
    }

    public static class shooter { //TODO shooter control constants
        public static final double ENCODER_RPM_PER_WHEEL_RPM = 0;
        public static final double kP = 1;
        public static final double kD = 1;
        public static final double kSpeakerRPM = 100;
        public static final double kAmpRPM = 30;
        public static final double kOff = 0;
    }
}
