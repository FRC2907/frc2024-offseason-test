package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.units.Units;

public class MechanismDimensions {
    
    public static class electrical {
        public static final double MAX_VOLTAGE = 12;
    }

    public static class arm {
        public static final double GEAR_RATIO = 1; //TODO find
        public static final double kLength = 1; //TODO find length of arm inches
        public static final double kHeight = 1; //TODO find height of arm above ground inches
    }

    public static class drivetrain {
        //https://www.andymark.com/products/toughbox-micro-s
        public static final double GEAR_RATIO = 1 / 5.95;  
        public static final double WHEEL_DIAMETER = 8; //inches

        //22.875 in. edge to edge wide, 20.5 in. edge to edge long
        public static final Translation2d FRONT_LEFT_LOCATION  = new Translation2d(Units.Meters.of(0.26035), Units.Meters.of(0.2905125));
        public static final Translation2d REAR_LEFT_LOCATION   = new Translation2d(Units.Meters.of(-0.26035), Units.Meters.of(0.2905125));
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(Units.Meters.of(0.26035), Units.Meters.of(-0.2905125));
        public static final Translation2d REAR_RIGHT_LOCATION  = new Translation2d(Units.Meters.of(-0.26035), Units.Meters.of(-0.2905125));

        public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
            FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, REAR_LEFT_LOCATION, REAR_RIGHT_LOCATION);
    }

    public static class intake {
        public static final double GEAR_RATIO = 1; //TODO find
        public static final double WHEEL_DIAMETER = 2; //inches
    }

    public static class shooter {
        public static final double GEAR_RATIO = 1; //TODO find
        public static final double WHEEL_DIAMETER = 4; //TODO find diameter
    }
}
