package frc.robot;

import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public class DrivetrainConst {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    public class BallTunnelConsts {
        public static final double MainGearRatio = 3;
        public static final double DivergentGearRatio = 1;
    }

    // Intake
    public class IntakeRollersConsts {
        public static final double GearRatio = 1; // slow down
    }
    // Actuation
    public class IntakeActuationConsts {
        public static final double GearRatio = 14.2; // slow down
    }

    // Shooter
    public class ShooterConsts {
        public static final double GearRatio = 1.2; // slow down
    }

    // AngleController
    public class AngleControllerConsts {
        public static final double ANGLE_CONTROLLER_REST_POS = 0;
        public static final double ANGLE_CONTROLLER_TEST_POS = 45;
        public static final double GEAR_RATIO = 2; // slow down
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

        public static final double LIMELIGHT_HEIGHT_1 = 0.1;
        //height above ground of camera, NOT ON ROBOT

        public static final double LIMELIGHT_ANGLE = 0;
        //angle of camera from looking straight, NOT ON ROBOT
    
        public static final String LIMELIGHT_NAME = "limelight";
    }

    public class FieldConst {
        public static final Pose2d BLUE_HUB = new Pose2d(4.03, 4.035, new Rotation2d(0));
        public static final Pose2d RED_HUB = new Pose2d(12.51, 4.035, new Rotation2d(0));

        public static final Pose2d RED_CORRAL = new Pose2d(16.54, 8.07 - 0.85, new Rotation2d(0));
        public static final Pose2d BLUE_CORRAL = new Pose2d(0, 0.85, new Rotation2d(0));
        
        public static final Pose2d RED_DEPOT = new Pose2d(16.2352, 2.159, new Rotation2d(0));
        public static final Pose2d BLUE_DEPOT = new Pose2d(0.3048, 8.07 - 2.159, new Rotation2d(0));
    }
}
