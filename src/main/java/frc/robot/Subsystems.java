package frc.robot;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeActuation;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.BallTunnel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeRollers;

public class Subsystems {
    public static IntakeActuation m_intakeActuation = new IntakeActuation(19);
    public static BallTunnel m_ballTunnel = new BallTunnel(17, 18);
    public static Shooter m_shooter = new Shooter(13, 14, 15);
    public static AngleController m_AngleController = new AngleController(16);
    public static IntakeRollers m_intakeRollers = new IntakeRollers(20); //todo need to sensor
    
    public static final LimelightSubsystem m_limelight = new LimelightSubsystem("limelight-front");
    public static final LimelightSubsystem m_limelight2 = new LimelightSubsystem("limelight-back");
}
