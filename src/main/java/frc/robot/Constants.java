package frc.robot;

import frc.robot.generated.TunerConstants;
import frc.robot.util.interpolable.InterpolatingDouble;
import frc.robot.util.interpolable.InterpolatingTreeMap;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class DrivetrainConst {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                  // desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a
                                                                                                      // rotation per
                                                                                                      // second max
                                                                                                      // angular
                                                                                                      // velocity
    }

    public class BallTunnelConsts {
        public static final double MainGearRatio = 3;
        public static final double DivergentGearRatio = 1;
        public static final double HopperGearRatio = 3;
    }

    // Intake
    public class IntakeRollersConsts {
        public static final double GearRatio = 4; // slow down
    }

    // Actuation
    public class IntakeActuationConsts {
        // 20 -> 40
        public static final double GearRatio = 51 / 4.4444;
        // 14.2; // slow down
    }

    // Shooter
    public class ShooterConsts {
        public static final double GearRatio = 1.2; // slow down
    }

    // AngleController
    public class AngleControllerConsts {
        public static final double ANGLE_CONTROLLER_REST_POS = 0;
        public static final double ANGLE_CONTROLLER_TEST_POS = 45;
        public static final double GEAR_RATIO = 36;
        public static final double CURRENT_TOLERANCE = 2;
    }

    public class VisionConsts {
        // to the bottom in meters

        public static final double UP_TO_CORRAL_TAG = 0.4921;
        // 13 + 14

        public static final double UP_TO_TRENCH_TAG = 0.7906;
        // 7 + 12

        public static final double UP_TO_HUB_TAG = 1.067;
        // 2 + 3 + 4 + 5 + 8 + 9 + 10 + 11

        public static double DIST_TO_STOP = 0.5;
        // distance away we want to stop from the april tags in meters

        public static final double LIMELIGHT_HEIGHT_1 = 0.1;
        // height above ground of camera, NOT ON ROBOT

        public static final double LIMELIGHT_ANGLE = 0;
        // angle of camera from looking straight, NOT ON ROBOT

        public static final String LIMELIGHT_NAME = "limelight";
    }

    public class FieldConst {
        public static final Pose2d BLUE_HUB = new Pose2d(4.611624, 4.034536, new Rotation2d(0));
        public static final Pose2d RED_HUB = new Pose2d(11.86815, 4.034536, new Rotation2d(0));

        public static final Pose2d RED_CORRAL = new Pose2d(16.54, 8.07 - 0.85, new Rotation2d(0));
        public static final Pose2d BLUE_CORRAL = new Pose2d(0, 0.85, new Rotation2d(0));

        public static final Pose2d RED_DEPOT = new Pose2d(16.2352, 2.159, new Rotation2d(0));
        public static final Pose2d BLUE_DEPOT = new Pose2d(0.3048, 8.07 - 2.159, new Rotation2d(0));

        public static final double kHubHeight = Units.inchesToMeters(72);
        public static final double kHubDistanceFromAlliance = Units.inchesToMeters(158.6);
        public static final double kHubDistanceLeftEdge = Units.inchesToMeters(138.65);
        public static final double kHubDistanceRightEdge = Units.inchesToMeters(185.65);
        public static final Pose3d kHubTarget = new Pose3d(kHubDistanceFromAlliance + 20,
                (kHubDistanceLeftEdge + kHubDistanceRightEdge) / 2, kHubHeight, new Rotation3d());

        public static final double kFuelWeight = Units.lbsToKilograms(0.474); // Kg

    }

    public static class ShootingConst {
        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> DISTANCE_TO_SHOOT_SPEED = new InterpolatingTreeMap<>();
        static {
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(17.0), new InterpolatingDouble(40.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(17.0), new InterpolatingDouble(45.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(17.0), new InterpolatingDouble(50.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(20.0), new InterpolatingDouble(55.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(20.0), new InterpolatingDouble(60.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(20.0), new InterpolatingDouble(65.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(23.0), new InterpolatingDouble(70.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(23.0), new InterpolatingDouble(75.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(23.0), new InterpolatingDouble(80.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(26.0), new InterpolatingDouble(85.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(26.0), new InterpolatingDouble(90.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(26.0), new InterpolatingDouble(95.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(29.0), new InterpolatingDouble(100.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(29.0), new InterpolatingDouble(105.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(29.0), new InterpolatingDouble(110.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(32.0), new InterpolatingDouble(115.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(32.0), new InterpolatingDouble(120.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(32.0), new InterpolatingDouble(125.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(32.0), new InterpolatingDouble(130.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(35.0), new InterpolatingDouble(135.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(35.0), new InterpolatingDouble(140.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(35.0), new InterpolatingDouble(145.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(35.0), new InterpolatingDouble(150.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(37.0), new InterpolatingDouble(155.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(37.0), new InterpolatingDouble(160.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(39.0), new InterpolatingDouble(165.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(39.0), new InterpolatingDouble(170.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(39.0), new InterpolatingDouble(175.0));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(40.0), new InterpolatingDouble(180.0));
        }
    }
}
