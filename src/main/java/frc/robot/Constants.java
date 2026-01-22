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

    public class VisionConsts {
        public static final double maxDistanceAway = 0.61;

        //all of the april tag heights from the top of the black

        //trenches
        public static final double ID_12_HEIGHT = .952;
        public static final double ID_7_HEIGHT = .952;
        public static final double ID_1_HEIGHT = .956;
        public static final double ID_6_HEIGHT = .956;

        //corral april tags
        public static final double ID_13_HEIGHT = .683;
        public static final double ID_14_HEIGHT = .683;

        //hub april tags
        public static final double ID_9_HEIGHT = 1.229;
        public static final double ID_10_HEIGHT = 1.229;
        public static final double ID_3_HEIGHT = 1.229;
        public static final double ID_4_HEIGHT = 1.229;
        public static final double ID_8_HEIGHT = 1.229;
        public static final double ID_2_HEIGHT = 1.229;
        public static final double ID_5_HEIGHT = 1.229;
        public static final double ID_11_HEIGHT = 1.229;

        
    }

    // Intake
    public class IntakeConsts {

    }
    // Actuation
    public class ActuationConsts {
        public static final double GearRatio = 1;
        public static final double DOWN_POS = 90;
        public static final double UP_POS = 0;
    }

    // Shooter
    public class ShooterConsts {

    }

    // AngleController
    public class AngleControllerConsts {
        public static final double ANGLE_CONTROLLER_REST_POS = 0;
        public static final double GEAR_RATIO = 1;
    }
}
