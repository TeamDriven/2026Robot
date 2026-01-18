package frc.robot;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Actuation;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.BallTunnel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

public class Subsystems {
    public static Actuation m_actuation = new Actuation(16);
    public static BallTunnel m_ballTunnel = new BallTunnel(13);
    public static Shooter m_shooter = new Shooter(14, 15);
    public static Indexer m_indexer = new Indexer(17);
    public static AngleController m_AngleController = new AngleController(18);
    
    public static final LimelightSubsystem m_limelight = new LimelightSubsystem();
}
