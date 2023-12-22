package frc.robot.Utils;

public interface Consts {
    public class SpeedValues{
        public static final double MAX_SPEED = 1; //max drive speed in m/s
    }

    public class PIDValues{
        public static final double WHEEL_ANGLE_KP = 0;
        public static final double WHEEL_ANGLE_KI = 0;
        public static final double WHEEL_ANGLE_KD = 0;

        public static final double WHEEL_VELOCITY_KP = 0;
        public static final double WHEEL_VELOCITY_KI = 0;
        public static final double WHEEL_VELOCITY_KD = 0;
        public static final double WHEEL_VELOCITY_KF = 0;
        
        public static final double HEADING_KP = 0;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;
        public static final double HEADING_TOLERANCE = 3;
    }
    
    public class MotorValues{
        //chassis motors
        public static final int TOP_LEFT_DRIVE_ID = 6;
        public static final int TOP_RIGHT_DRIVE_ID = 8;
        public static final int DOWN_LEFT_DRIVE_ID = 4;
        public static final int DOWN_RIGHT_DRIVE_ID = 10;
        
        public static final double DRIVE_GEAR_RATIO = 1 / 6.75;

        public static final int TOP_LEFT_STEERING_ID = 7;
        public static final int TOP_RIGHT_STEERING_ID = 9;
        public static final int DOWN_LEFT_STEERING_ID = 5;
        public static final int DOWN_RIGHT_STEERING_ID = 11;

        public static final double STEERING_GEAR_RATIO = 1 / 12.8;

    }

    public class ChassisValues{
        public static final double WHEEL_PERIMETER = Math.PI * 0.0935;
        
        public static final double FRONT_WHEELS_DISTANCE = 0.57;
        public static final double SIDE_WHEELS_DISTANCE = 0.57;
        //create modules position vectors 
        public static final Vector2d TOP_LEFT = new Vector2d(-SIDE_WHEELS_DISTANCE / 2, FRONT_WHEELS_DISTANCE / 2);
        public static final Vector2d TOP_RIGHT = new Vector2d(SIDE_WHEELS_DISTANCE / 2, FRONT_WHEELS_DISTANCE / 2);
        public static final Vector2d DOWN_LEFT = new Vector2d(-SIDE_WHEELS_DISTANCE / 2, -FRONT_WHEELS_DISTANCE / 2);
        public static final Vector2d DOWN_RIGHT = new Vector2d(SIDE_WHEELS_DISTANCE / 2, -FRONT_WHEELS_DISTANCE / 2);
        public static final Vector2d[] MODULES_POSITION_VECTORS = {TOP_LEFT, TOP_RIGHT, DOWN_LEFT, DOWN_RIGHT};        
    }

}
