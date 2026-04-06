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
        public static final double MAX_CURRENT_TOLERANCE = 5;
        public static final double MIN_CURRENT_TOLERANCE = .3;

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
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(40.0), new InterpolatingDouble(18));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(45.0), new InterpolatingDouble(18));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(50.0), new InterpolatingDouble(18));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(55.0), new InterpolatingDouble(21));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(60.0), new InterpolatingDouble(21));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(65.0), new InterpolatingDouble(21));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(70.0), new InterpolatingDouble(21));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(75.0), new InterpolatingDouble(22));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(80.0), new InterpolatingDouble(24));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(85.0), new InterpolatingDouble(24));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(90.0), new InterpolatingDouble(24));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(95.0), new InterpolatingDouble(25));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(100.0), new InterpolatingDouble(25));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(105.0), new InterpolatingDouble(25));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(110.0), new InterpolatingDouble(27)); // sdxzfszx
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(115.0), new InterpolatingDouble((27)));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(120.0), new InterpolatingDouble((27)));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(125.0), new InterpolatingDouble((27)));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(130.0), new InterpolatingDouble((28)));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(135.0), new InterpolatingDouble((29)));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(140.0), new InterpolatingDouble(29));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(145.0), new InterpolatingDouble(30));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(150.0), new InterpolatingDouble(30));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(155.0), new InterpolatingDouble(31));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(160.0), new InterpolatingDouble(31.5));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(165.0), new InterpolatingDouble(33));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(170.0), new InterpolatingDouble(34));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(175.0), new InterpolatingDouble(35));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(180.0), new InterpolatingDouble(35));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(185.0), new InterpolatingDouble(35));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(190.0), new InterpolatingDouble(36));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(195.0), new InterpolatingDouble(36));
            DISTANCE_TO_SHOOT_SPEED.put(new InterpolatingDouble(200.0), new InterpolatingDouble(36));
        }


        public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> DISTANCE_TO_SHOOT_ANGLE = new InterpolatingTreeMap<>();
        static {
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(40.0), new InterpolatingDouble(8.0));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(45.0), new InterpolatingDouble(8.0));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(50.0), new InterpolatingDouble(8.0));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(55.0), new InterpolatingDouble(9));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(60.0), new InterpolatingDouble(9));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(65.0), new InterpolatingDouble(9));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(70.0), new InterpolatingDouble(9));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(75.0), new InterpolatingDouble(9));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(80.0), new InterpolatingDouble(9));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(85.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(90.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(95.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(100.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(105.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(110.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(115.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(120.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(125.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(130.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(135.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(140.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(145.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(150.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(155.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(160.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(165.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(170.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(175.0), new InterpolatingDouble(10));
            DISTANCE_TO_SHOOT_ANGLE.put(new InterpolatingDouble(180.0), new InterpolatingDouble(12.0));
        }

    }
}
