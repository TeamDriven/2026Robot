// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Shooter class represents the subsystem responsible for controlling the
 * shooter mechanism.
 * It handles the initialization of motors, running the shooter at a given
 * velocity and acceleration,
 * and stopping the shooter.
 */
public class Shooter extends SubsystemBase {

  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;

  VelocityVoltage velocityControl;
  VelocityVoltage slowVelocityControl;
  NeutralOut stopMode;

  /**
   * Creates a new Intake.
   */
  public Shooter(int leftMotorId, int rightMotorId) {
    leftShooterMotor = new TalonFX(leftMotorId);
    rightShooterMotor = new TalonFX(rightMotorId);
    initMotors();

    velocityControl = new VelocityVoltage(0);

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
    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.kP = 0.4; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.35; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / Rotation per second

    configs.Slot1.kP = 0.278; // 0.278 // An error of 1 rotation per second results in 2V output
    configs.Slot1.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot1.kD = 0.0005; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot1.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // configs.CurrentLimits.SupplyCurrentLimit = 11;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightShooterMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leftShooterMotor.getConfigurator().apply(configs);
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
    leftShooterMotor.setControl(velocityControl
        .withVelocity(velocity)
        .withAcceleration(acceleration));

    rightShooterMotor.setControl(velocityControl
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
        leftShooterMotor.set(speed);
        rightShooterMotor.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
      }
    };
  }

  public void stopMotors() {
    leftShooterMotor.setControl(stopMode);
    rightShooterMotor.setControl(stopMode);
  }

  /**
   * Get the left shooter velocity
   * 
   * @return a double representing the left shooter velocity
   */
  public double getLeftVelocity() {
    return leftShooterMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Get the right shooter velocity
   * 
   * @return a double representing the right shooter velocity
   */
  public double getRightVelocity() {
    return rightShooterMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // System.out.println("left: " + leftShooterMotor.getVelocity().getValueAsDouble());
    // System.out.println("right: " + rightShooterMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {

  }
}