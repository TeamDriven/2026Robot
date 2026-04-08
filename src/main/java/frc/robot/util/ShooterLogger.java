package frc.robot.util;
import dev.doglog.DogLog;
import frc.robot.Constants.DrivetrainConst;
import frc.robot.Telemetry;


public class ShooterLogger extends DogLog {

   public static void log(double velocity, double targetvelocity){
    DogLog.log("shooter target vel", targetvelocity);
    DogLog.log("shooter actual vel", velocity);
   } 
}
