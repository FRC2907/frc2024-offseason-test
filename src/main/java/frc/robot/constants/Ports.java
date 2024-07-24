package frc.robot.constants;

public class Ports {
    public static class can { //TODO all can ids are currently placeholders
        public static class arm {
            public static final int[] MOTORS = { 5, 6 };
        }
        
        public static class drivetrain {
            public static final int FRONT_LEFT = 1;
            public static final int REAR_LEFT = 2;
            public static final int FRONT_RIGHT = 3;
            public static final int REAR_RIGHT = 4;
        }

        public static class intake {
            public static final int[] MOTORS = { 13, 12 };
        }

        public static class shooter {
            public static final int[] MOTORS = { 8, 7 };
        }
    }

    public static class PWM {
        public static final int LED = 0;
        
    }

    public static class HID {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }
}