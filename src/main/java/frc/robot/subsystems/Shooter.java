// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_limelight;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConsts;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ShooterLogger;
import frc.robot.util.interpolable.InterpolatingDouble;

import static frc.robot.RobotContainer.xPose;
import static frc.robot.RobotContainer.yPose;

/**
 * The Shooter class represents the subsystem responsible for controlling the
 * shooter mechanism.
 * It handles the initialization of motors, running the shooter at a given
 * velocity and acceleration,
 * and stopping the shooter.
 */
public class Shooter extends SubsystemBase {

  private TalonFX leftShooterMotor;
  private TalonFX rightTopShooterMotor;
  private TalonFX rightBottomShooterMotor;

  VelocityVoltage velocityControl;
  VelocityVoltage slowVelocityControl;
  NeutralOut stopMode;

  /**
   * Creates a new Shooter.
   */
  public Shooter(int leftMotorId, int rightBottomMotorId, int rightTopMotorId) {
    leftShooterMotor = new TalonFX(leftMotorId, TunerConstants.kCANBus);
    rightTopShooterMotor = new TalonFX(rightTopMotorId, TunerConstants.kCANBus);
    rightBottomShooterMotor = new TalonFX(rightBottomMotorId, TunerConstants.kCANBus);
    initMotors();

    velocityControl = new VelocityVoltage(0).withEnableFOC(true).withFeedForward(4.5);

    // sitControl = new VoltageOut(2);

    stopMode = new NeutralOut();
  }

  /**
   * Initialize the both shooter motors
   */
  public void initMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.Feedback.SensorToMechanismRatio = ShooterConsts.GearRatio;

    leftShooterMotor.setControl(new Follower(rightTopShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    rightBottomShooterMotor.setControl(new Follower(rightTopShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.kP = 1; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.00; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12

    // volts / Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // configs.CurrentLimits.SupplyCurrentLimit = 11;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightTopShooterMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leftShooterMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightBottomShooterMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * 
   * @param velocity     in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the intake motor
   */
  public Command runShooterCommand(double acceleration) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Shooter.this);
      }

      @Override
      public void execute() {
        double velocity = calcSpeedV2(xPose, yPose).getAsDouble();
        runShooter(velocity, acceleration);
       ShooterLogger.log(getVelocity(), velocity);

      }

      @Override
      public void end(boolean interrupted) {
        // sitMode();
       // stopMotors();
      }

      @Override
      public boolean isFinished(){
          double velocity = calcSpeedV2(xPose, yPose).getAsDouble();
double motorVelocity = getVelocity();
      SmartDashboard.putNumber("shooter speed ", motorVelocity);
      return Math.abs(motorVelocity - velocity) <= 4;      }
    };
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * 
   * @param velocity     in rotations per second
   * @param acceleration in rotations per second squared
   */
  public void runShooter(double velocity, double acceleration) {

    rightTopShooterMotor.setControl(velocityControl
        .withVelocity(velocity)
        .withAcceleration(acceleration));
  }


  /**
   * Run the Shooting motors at a given percent
   * 
   * @param speed 1 to -1
   * @return a command that will run the intake motor
   */
  public Command runShooterPercent(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Shooter.this);
      }

      @Override
      public void execute() {
        rightTopShooterMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        rightTopShooterMotor.set(0);
      }
    };
  }

  public void stopMotors() {
    rightTopShooterMotor.setControl(stopMode);
  }

  /**
   * Get the left shooter velocity
   * 
   * @return a double representing the left shooter velocity
   */
  public double getVelocity() {
    return rightTopShooterMotor.getVelocity().getValueAsDouble();
  }

  public Command waitUntilAtSpeed(double speed) {
    return new WaitUntilCommand(() -> {
      double motorVelocity = getVelocity();
      SmartDashboard.putNumber("shooter speed ", motorVelocity);
      return Math.abs(motorVelocity - speed) <= 2; // 1 degree tolerance
    });
  }

  public Command stopShooterCommand(){
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Shooter.this);
      }

      @Override
      public void execute() {
        stopMotors();
      }

      @Override
      public void end(boolean interrupted) {
        stopMotors();
      }
    };
  }

  public double calcSpeed() {
    double loss = 1.25;
    double g = 9.81; // Acceleration due to gravity in m/s^2
    // double angleRadians = Math.toRadians(Constants.RobotConstants.kShooterAngle);

    double angleRadians = Math.toRadians(m_angleController.calculateHoodAngle());

    double d = Constants.FieldConst.kHubTarget.getX() - m_limelight.getMegaTag2().pose.getX(); // Horizontal distance to
                                                                                               // target

    double top = d * g;
    double bottom = Math.sin(2 * angleRadians);

    return Math.sqrt(loss * top / bottom);
  }

  public DoubleSupplier calcSpeedV2(double x, double y) {
    double curentXDistance = 0;
    double curentYDistance = 0;

    if (DriverStation.getAlliance().get() == Alliance.Red) {
    curentXDistance = Math.pow(Constants.FieldConst.RED_HUB.getX() - x, 2);
    curentYDistance = Math.pow(Constants.FieldConst.RED_HUB.getY() - y, 2);
    } else {
      curentXDistance = Math.pow(Constants.FieldConst.BLUE_HUB.getX() - x, 2);
      curentYDistance = Math.pow(Constants.FieldConst.BLUE_HUB.getY() - y, 2);
    }
    // curentXDistance = Math.pow(Constants.FieldConst.BLUE_HUB.getX() -
    // m_limelight.getMegaTag2().pose.getX(), 2);
    // curentYDistance = Math.pow(Constants.FieldConst.BLUE_HUB.getY() -
    // m_limelight.getMegaTag2().pose.getY(), 2);
    // }

 double limelightX = m_limelight.getMegaTag2().pose.getX();
 double limelightY = m_limelight.getMegaTag2().pose.getY();
 double limelightXDistance = Math.pow(Constants.FieldConst.BLUE_HUB.getX() - limelightX, 2);
 double limelightYDistance = Math.pow(Constants.FieldConst.BLUE_HUB.getY() - limelightY, 2);
 double limelightDistance = Math.sqrt(limelightYDistance + limelightXDistance);
 SmartDashboard.putNumber(("limelightDistance"), limelightDistance);

    double distance = Math.sqrt(curentYDistance + curentXDistance);
    double speed = Constants.ShootingConst.DISTANCE_TO_SHOOT_SPEED
        .getInterpolated(new InterpolatingDouble(Units.metersToInches(distance))).value;
    SmartDashboard.putNumber("CalcSpeed.spd", speed);
    SmartDashboard.putNumber("CalcSpeed.dist",Units.metersToInches(distance));
    SmartDashboard.putNumber("CalcSpeed.XDist", curentXDistance);
    SmartDashboard.putNumber("CalcSpeed.YDist", curentYDistance);
    SmartDashboard.putNumber("getXPose", x);
    SmartDashboard.putNumber("getYPose", y);
    return () -> speed;
  }

  @Override
  public void periodic() {
    // double curentXDistance = Math.pow(Constants.FieldConst.RED_HUB.getX() - RobotContainer.drivetrain.getPose().getX(), 2);
    // System.out.println("distance Meters X: " + curentXDistance);
    // double curentYDistance = Math.pow(Constants.FieldConst.RED_HUB.getY() - RobotContainer.drivetrain.getPose().getX(), 2);
    // System.out.println("distance Meters Y: " + curentYDistance);
    // System.out.println("distance Meters: " + Math.sqrt(curentYDistance + curentXDistance));
    // // System.out.println("distance Inches: " + Units.metersToInches(distance));
    // System.out.println("Shooter speed: " + leftShooterMotor.getVelocity());
  }

}