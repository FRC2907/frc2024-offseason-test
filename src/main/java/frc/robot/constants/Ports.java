package frc.robot.constants;

public class Ports {
    public static class can {
        public static class drivetrain {
            public static final int[] LEFTS = { 1, 2, 3 };
            public static final int[] RIGHTS = { 14, 15, 16 };
        }

        public static class intake {
            public static final int MOTOR = 13;
        }

        public static class shooter {
            public static final int LEFT = 7;
            public static final int RIGHT = 8;

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
