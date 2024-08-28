package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain.DriveMode;

public class Control {
    public static final double kZeroHysteresis = 2;
    public static final double kInchesPerMinuteToMetersPerSecond = 1 / (60 /*seconds*/ * Units.metersToInches(1));
    
    public static class arm { //TODO fix/measure arm control constants
        public static final double ENCODER_POS_UNIT_PER_DEGREE = 0; // tick/deg
        public static final double ENCODER_VEL_UNIT_PER_DEGREE_PER_SECOND = 0; //tick/deg/s

        public static final double kVelocityConversionFactor = 
                          (1 / kInchesPerMinuteToMetersPerSecond) 
                        / (MechanismDimensions.arm.kLength * 2 * Math.PI)
                        *  MechanismDimensions.arm.GEAR_RATIO;
        public static final double kPositionConversionFactor = MechanismDimensions.arm.GEAR_RATIO 
                                                             * 360;
                                                            
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
        public static final double kVelocityHysteresis = 100;
    }

    public static class drivetrain {
        public static final DriveMode kDefaultDriveMode = DriveMode.LOCAL_FORWARD;

        public static final double kVelocityConversionFactor = 
                          (1 / kInchesPerMinuteToMetersPerSecond) 
                        / (MechanismDimensions.drivetrain.WHEEL_DIAMETER * Math.PI)
                        *  MechanismDimensions.drivetrain.GEAR_RATIO;
                        
    }

    public static class intake { //TODO fix/measure intake control constants
        public static final double ENCODER_VEL_UNIT_PER_INTAKE_MPS = 0;
        public static final double ENCODER_AMPS_PER_INTAKE_MPS = 0;

        public static final double kVelocityConversionFactor = 
                          (1 / kInchesPerMinuteToMetersPerSecond) 
                        / (MechanismDimensions.intake.WHEEL_DIAMETER * Math.PI)
                        *  MechanismDimensions.intake.GEAR_RATIO;
        public static final double kP = 1;
        public static final double kD = 1;

        public static final double kIntakingSpeed = 10; // m/s
        public static final double kOff = 0;
        public static final double kOutakingSpeed = -12;
        public static final double kShoot = 5;
        public static final double kVelocityHysteresis = 100;
        public static final double kOnHysteresis = 3;
        public static final double kAverageCurrent = 0;
        public static final double kCurrentHystereis = 2;

        public static final int kArrayLength = 50;
    }

    public static class shooter { //TODO fix/measure shooter control constants
        public static final double ENCODER_VEL_UNIT_PER_SHOOTER_MPS = 0;

        public static final double kVelocityConversionFactor = 
                          (1 / kInchesPerMinuteToMetersPerSecond) 
                        / (MechanismDimensions.shooter.WHEEL_DIAMETER * Math.PI)
                        *  MechanismDimensions.shooter.GEAR_RATIO;
        public static final double kP = 1;
        public static final double kD = 1;

        public static final double kSpeakerSpeed = 30; // m/s
        public static final double kAmpSpeed = 10;
        public static final double kOff = 0;
        public static final double kVelocityHysteresis = 100;
        public static final double kMaxSpeed = 100;
        public static final double kAverageCurrent = 0;
        public static final double kCurrentHystereis = 2;

        public static final int kArrayLength = 50;
    }



    public static class led { //all color values found here: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
        public static final double red = 0.61;
        public static final double orange = 0.65;
        public static final double yellow = 0.69;
        public static final double green = 0.77;
        public static final double blue = 0.87;
        public static final double violet = 0.91;
        public static final double white = 0.93;
        public static final double black = 0.99;
    }
}