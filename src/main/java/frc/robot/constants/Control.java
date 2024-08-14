package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain.DriveMode;

public class Control {
    public static final double kZeroHysteresis = 2;
    public static final double kInchesPerMinuteToMetersPerSecond = 1 / (60 /*seconds*/ * Units.metersToInches(1));
    
    public static class arm { //TODO arm control constants
        public static final double ENCODER_POS_UNIT_PER_DEGREE = 0; // tick/deg
        public static final double ENCODER_VEL_UNIT_PER_DEGREE_PER_SECOND = 0; //tick/deg/s

        public static final double kVelocityConversionFactor = 
                          (1 / kInchesPerMinuteToMetersPerSecond) 
                        / (MechanismDimensions.arm.kLength * 2 * Math.PI);
        public static final double kP = 1;
        public static final double kD = 1;

        public static final double kStartPosition = 0; //deg 
        public static final double kAmpPosition = 50;     
        public static final double kIntakePosition = 30;
        public static final double kHoldingPosition = 50;
        public static final double kNeutralPosition = 30;
        public static final double kSubwooferPosition = 60;
        public static final double kWingPosition = 30;
        public static final double kMinPosition = 0;
        public static final double kMaxPosition = 75;

        public static final double kManualControlDiff = 2;
        public static final double kPositionHysteresis = 2;
        public static final double kVelocityHysteresis = 2;
    }

    public static class drivetrain {
        public static final DriveMode kDefaultDriveMode = DriveMode.LOCAL_FORWARD;

        public static final double kVelocityConversionFactor = 
                          (1 / kInchesPerMinuteToMetersPerSecond) 
                        / (MechanismDimensions.drivetrain.WHEEL_DIAMETER * Math.PI);
                        
    }

    public static class intake { //TODO intake control constants
        public static final double ENCODER_VEL_UNIT_PER_INTAKE_MPS = 0;
        public static final double ENCODER_AMPS_PER_INTAKE_MPS = 0;

        public static final double kVelocityConversionFactor = 
                          (1 / kInchesPerMinuteToMetersPerSecond) 
                        / (MechanismDimensions.intake.WHEEL_DIAMETER * Math.PI);
        public static final double kP = 1;
        public static final double kD = 1;

        public static final double kIntakingSpeed = 5; // m/s
        public static final double kOff = 0;
        public static final double kOutakingSpeed = -5;
        public static final double kShoot = 3;
        public static final double kVelocityHysteresis = 2;
        public static final double kOnHysteresis = 3;
        public static final double kAverageCurrent = 0;
        public static final double kCurrentHystereis = 2;

        public static final int kArrayLength = 50;
    }

    public static class shooter { //TODO shooter control constants
        public static final double ENCODER_VEL_UNIT_PER_SHOOTER_MPS = 0;

        public static final double kVelocityConversionFactor = 
                          (1 / kInchesPerMinuteToMetersPerSecond) 
                        / (MechanismDimensions.shooter.WHEEL_DIAMETER * Math.PI);
        public static final double kP = 1;
        public static final double kD = 1;

        public static final double kSpeakerSpeed = 10; // m/s
        public static final double kAmpSpeed = 3;
        public static final double kOff = 0;
        public static final double kVelocityHysteresis = 2;
        public static final double kMaxSpeed = 10;
        public static final double kAverageCurrent = 0;
        public static final double kCurrentHystereis = 2;
    }
}