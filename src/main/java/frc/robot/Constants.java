package frc.robot;

import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Constants {
    public class DrivetrainConst {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    // Intake
    public class IntakeConsts {

    }
    // Actuation
    public class ActuationConsts {
        public static final double GearRatio = 1;
    }

    // Shooter
    public class ShooterConsts {

    }

    // AngleController
    public class AngleControllerConsts {
        public static final double ANGLE_CONTROLLER_REST_POS = 0;
        public static final double GEAR_RATIO = 1;
    }
    public class VisionConsts {
        //to the bottom in meters

        public static final double UP_TO_CORRAL_TAG = 0.4921;
        //13 + 14

        public static final double UP_TO_TRENCH_TAG = 0.7906;
        //7 + 12

        public static final double UP_TO_HUB_TAG = 1.067;
        //2 + 3 + 4 + 5 + 8 + 9 + 10 + 11

        public static double DIST_TO_STOP = 0.5;
        // distance away we want to stop from the april tags in meters

        public static final double LIMELIGHT_HEIGHT = 0.1;
        //height above ground of camera, NOT ON ROBOT

        public static final double LIMELIGHT_ANGLE = 0;
        //angle of camera from looking straight, NOT ON ROBOT
    
        public static final String LIMELIGHT_NAME = "limelight";
    }
}
