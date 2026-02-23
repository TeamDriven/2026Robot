// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollersConsts;
import frc.robot.Constants.ShooterConsts;
import frc.robot.generated.TunerConstants;

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
   * Creates a new Intake.
   */
  public Shooter(int leftMotorId, int rightBottomMotorId,  int rightTopMotorId) {
    leftShooterMotor = new TalonFX(leftMotorId, TunerConstants.kCANBus);
    rightTopShooterMotor = new TalonFX(rightTopMotorId, TunerConstants.kCANBus);
    rightBottomShooterMotor = new TalonFX(rightBottomMotorId, TunerConstants.kCANBus);
    initMotors();

    velocityControl = new VelocityVoltage(0).withEnableFOC(true);

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
    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.kP = 0.15; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
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

    leftShooterMotor.setControl(new Follower(rightTopShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    rightBottomShooterMotor.setControl(new Follower(rightTopShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  /**
   * Run the intake motor at a given velocity and acceleration
   * 
   * @param velocity     in rotations per second
   * @param acceleration in rotations per second squared
   * @return a command that will run the intake motor
   */
  public Command runShooterCommand(double velocity, double acceleration) {
    return new Command() {
      @Override
      public void initialize() {
        addRequirements(Shooter.this);
      }

      @Override
      public void execute() {
        runShooter(velocity, acceleration);
      }

      @Override
      public void end(boolean interrupted) {
        // sitMode();
        stopMotors();
      }
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

  @Override
  public void periodic() {
    System.out.println("left: " + leftShooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("right Bottom vel: " ,rightTopShooterMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {

  }
}