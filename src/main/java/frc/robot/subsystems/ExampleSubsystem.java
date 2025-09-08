// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ExampleSubsystem extends SubsystemBase implements Reportable{
  private final TalonFX wheely;
  private TalonFXConfigurator wheelyConfigurator;
  private boolean enabled = false;
  private NeutralModeValue neutralMode = NeutralModeValue.Brake;
  private double desiredSpeed = 0.0;

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.MEDIUM;
  // private double desiredPosition = 0.0;
  // private final Follower followRequest;
  // private MotionMagicVoltage motionMagicRequest;
  // private double feedforward = 0.0;
  // private double pivotAngle = 0.0;

  // private double kP = 3.5;
  // private double kG = 2.0;
  // private double cruiseVelocity = 1.0;
  // private double accleration = 0.5;
  // private double jerk = 0.1;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    wheely = new TalonFX(1,"rio");
    // motionMagicRequest = new MotionMagicVoltage(0);

    wheely.setPosition(0.0);

    wheelyConfigurator = wheely.getConfigurator();

    setMotorConfigs();

    // followRequest = new Follower(1,true);
    // motionMagicRequest.withSlot(0);
    zeroEncoders();
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public void setMotorConfigs(){
    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    wheelyConfigurator.refresh(motorConfigs);
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    motorConfigs.Feedback.SensorToMechanismRatio = 16;
    motorConfigs.Feedback.RotorToSensorRatio = 1;

    motorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 45;
    motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0.1;

    motorConfigs.MotorOutput.NeutralMode = neutralMode;
    motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // motorConfigs.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    // motorConfigs.MotionMagic.MotionMagicAcceleration = accleration;
    // motorConfigs.MotionMagic.MotionMagicJerk = jerk;

    // motorConfigs.Slot0.kP = kP;
    // motorConfigs.Slot0.kG = 0;
    // motorConfigs.Slot0.kS = 0;
  }

  public void zeroEncoders(){
    wheely.setPosition(0.0);
    // desiredPosition = 0.0;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    if (!enabled){
      wheely.set(0.0);
      return;
    }

    wheely.set(desiredSpeed);
  }

  public double getTargetSpeed() {
    return desiredSpeed;
  }

  public double getCurrentSpeed() {
    return wheely.get();
  }

  public void setSpeed(double speed){
    desiredSpeed = speed;
  }

  public void setEnabled(boolean request){
    enabled = request;
  }

  public boolean atSpeed(){
    return wheely.get() >= desiredSpeed;
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //logging
  @Override
  public void initShuffleboard(LOG_LEVEL priority){
    if (priority == LOG_LEVEL.OFF){
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Wheely");

    switch (priority){
      case OFF:
        break;
      case ALL:
        tab.addDouble("Current Speed", () -> getCurrentSpeed());
        tab.addBoolean("At Speed", () -> atSpeed());
      case MEDIUM:
        tab.addDouble("Target Speed", () -> getTargetSpeed());
        tab.addBoolean("Enabled", () -> enabled);
      case MINIMAL:
        tab.addDouble("Current Voltage", () -> wheely.getMotorVoltage().getValueAsDouble());
        tab.addDouble("Current Temperature", () -> wheely.getDeviceTemp().getValueAsDouble());

        break;
    }
  }

  @Override
  public void reportToSmartDashboard(LOG_LEVEL priority){
    if (priority == LOG_LEVEL.OFF){
      return;
    }

    switch (priority){
      case OFF:
        break;
      case ALL:
        SmartDashboard.putNumber("Current Speed", getCurrentSpeed());
        SmartDashboard.putBoolean("At Speed", atSpeed());
      case MEDIUM:
        SmartDashboard.putNumber("Target Speed", getTargetSpeed());
        SmartDashboard.putBoolean("Enabled", enabled);
      case MINIMAL:
        SmartDashboard.putNumber("Current Voltage", wheely.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Current Temperature", wheely.getDeviceTemp().getValueAsDouble());

        break;
    }
  }
}
