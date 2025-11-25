// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** IO Layer implementation for TalonFX drive/ NEO turn on SDS MK4i L2 swerve modules */
public class ModuleIOTBSwerve implements ModuleIO {

  private static final double DRIVE_GEAR_RATIO =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // L2 gearing
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final TalonFX driveTalon;
  private final SparkBase turnSparkMax;
  private final CANcoder cancoder;

  private final String moduleLabel;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;
  private final StatusSignal<Angle> turnAbsolutePosition;

  private final boolean isTurnMotorInverted = true;
  private InvertedValue isDriveMotorInverted = InvertedValue.CounterClockwise_Positive;
  private final Rotation2d absoluteEncoderOffset;

  /**
   * Create swerve module object, using hardware specific constants
   *
   * @param index
   */
  public ModuleIOTBSwerve(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(Constants.FRONT_LEFT_DRIVE_MOTOR);
        isDriveMotorInverted = Constants.FRONT_LEFT_DRIVE_MOTOR_INVERTED;
        turnSparkMax = new SparkMax(Constants.FRONT_LEFT_STEER_MOTOR, MotorType.kBrushless);
        cancoder = new CANcoder(Constants.FRONT_LEFT_STEER_ENCODER);
        absoluteEncoderOffset = new Rotation2d(Constants.FRONT_LEFT_STEER_OFFSET);
        moduleLabel = "Module0";
        break;
      case 1:
        driveTalon = new TalonFX(Constants.FRONT_RIGHT_DRIVE_MOTOR);
        isDriveMotorInverted = Constants.FRONT_RIGHT_DRIVE_MOTOR_INVERTED;
        turnSparkMax = new SparkMax(Constants.FRONT_RIGHT_STEER_MOTOR, MotorType.kBrushless);
        cancoder = new CANcoder(Constants.FRONT_RIGHT_STEER_ENCODER);
        absoluteEncoderOffset = new Rotation2d(Constants.FRONT_RIGHT_STEER_OFFSET);
        moduleLabel = "Module1";
        break;
      case 2:
        driveTalon = new TalonFX(Constants.BACK_LEFT_DRIVE_MOTOR);
        isDriveMotorInverted = Constants.BACK_LEFT_DRIVE_MOTOR_INVERTED;
        turnSparkMax = new SparkMax(Constants.BACK_LEFT_STEER_MOTOR, MotorType.kBrushless);
        cancoder = new CANcoder(Constants.BACK_LEFT_STEER_ENCODER);
        absoluteEncoderOffset = new Rotation2d(Constants.BACK_LEFT_STEER_OFFSET);
        moduleLabel = "Module2";
        break;
      case 3:
        driveTalon = new TalonFX(Constants.BACK_RIGHT_DRIVE_MOTOR);
        isDriveMotorInverted = Constants.BACK_RIGHT_DRIVE_MOTOR_INVERTED;
        turnSparkMax = new SparkMax(Constants.BACK_RIGHT_STEER_MOTOR, MotorType.kBrushless);
        cancoder = new CANcoder(Constants.BACK_RIGHT_STEER_ENCODER);
        absoluteEncoderOffset = new Rotation2d(Constants.BACK_RIGHT_STEER_OFFSET);
        moduleLabel = "Module3";
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    // Set current limit configs for drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.DRIVE_CURRENT_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = Constants.DRIVE_STATOR_CURRENT_LIMIT;
    driveTalon.getConfigurator().apply(driveConfig);

    setDriveBrakeMode(true, isDriveMotorInverted);

    var turnConfig = new SparkMaxConfig();
    turnConfig.idleMode(IdleMode.kBrake);
    turnConfig.voltageCompensation(12.0);
    turnConfig.inverted(false);
    turnConfig.smartCurrentLimit(20);
    turnSparkMax.configure(
        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Return initial status signals for Phoenix6 devices
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    turnSparkMax.setCANTimeout(0);

    // Set update frequencies for TalonFX status signals, position at 2x frequency
    // of other data
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);
    driveTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);

    // Report status code to AdvantageAlerts
    // AlertHandler.reportStatusCodeFault(drivePosition.getStatus(), moduleLabel,
    // driveMotorDisconnectAlert, driveMotorFirmwareAlert);
    // AlertHandler.reportSparkMaxFault(moduleLabel, turnSparkMax, turnMotorDisconnectAlert,
    // turnMotorCurrentAlert);

    // Drive motor inputs
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Turn motor / CANcoder positions
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnSparkMax.getEncoder().getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnSparkMax.getEncoder().getVelocity())
            / TURN_GEAR_RATIO;

    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
  }

  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  /**
   * Set inversion and brake mode for drive TalonFX
   *
   * @param enable
   * @param inversion
   */
  public void setDriveBrakeMode(boolean enable, InvertedValue inversion) {
    var config = new MotorOutputConfigs();
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.Inverted = inversion;
    driveTalon.getConfigurator().apply(config);
  }

  public TalonFX getDriveTalon() {
    return driveTalon;
  }
}
