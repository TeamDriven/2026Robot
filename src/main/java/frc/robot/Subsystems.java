package frc.robot;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Actuation;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.BallTunnel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class Subsystems {
    // public static Actuation m_actuation = new Actuation(17);
    // public static BallTunnel m_ballTunnel = new BallTunnel(14);
    // public static Shooter m_shooter = new Shooter(15, 16);
    // public static Indexer m_indexer = new Indexer(18);
    // public static AngleController m_AngleController = new AngleController(19);
    // public static Intake m_intake = new Intake(13); //todo need to sensor
    
    public static final LimelightSubsystem m_limelight = new LimelightSubsystem("limelight-front");
    public static final LimelightSubsystem m_limelight2 = new LimelightSubsystem("limelight-back");
}
