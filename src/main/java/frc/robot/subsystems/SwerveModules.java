// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModules extends SubsystemBase {
  public int moduleNumber;

  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private SparkMaxPIDController turnPIDController;
  private final SparkMaxPIDController driveController;
  private CANCoder absoluteEncoder;

  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.driveKS, Constants.driveKV, Constants.driveKA);

  /** Creates a new SwerveModule. */
  public SwerveModules(int moduleNumber, SwerveModuleConstants moduleConstants) {  //int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed
      this.moduleNumber = moduleNumber;
      angleOffset = moduleConstants.angleOffset;

      absoluteEncoder = new CANCoder(moduleConstants.cancoderID);
      configAngleEncoder();

      driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
      driveEncoder = driveMotor.getEncoder();
      driveController = driveMotor.getPIDController();
      driveMotor.setInverted(moduleConstants.driveMotorInverted);
      configDriveMotor();

      //driveMotor.setInverted(true);

      turnMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
      turnEncoder = turnMotor.getEncoder();
      turnPIDController = turnMotor.getPIDController();
      turnMotor.setInverted(moduleConstants.angleMotorInverted);
      configTurnMotor();

      lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    //desiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);

    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    Logger.getInstance().recordOutput("Drivetrain/Module " + driveMotor.getDeviceId() + " State", getState());
  }

  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.DRIVETRAIN_MAX_SPEED; //Max drivetrain speed
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DRIVETRAIN_MAX_SPEED * 0.01))
        ? lastAngle
        : desiredState.angle;

    turnPIDController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    turnEncoder.setPosition(absolutePosition);
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  private void configAngleEncoder() {
    absoluteEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(absoluteEncoder, CCUsage.kMinimal);
    absoluteEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configTurnMotor() {
    turnMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(turnMotor, Usage.kPositionOnly);
    turnMotor.setSmartCurrentLimit(30);
    //turnMotor.setInverted(true);
    turnMotor.setIdleMode(IdleMode.kCoast);
    turnEncoder.setPositionConversionFactor(Constants.TURN_MOTOR_PCONVERSION);
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(-180.0);
    turnPIDController.setPositionPIDWrappingMaxInput(180.0);
    turnPIDController.setP(Constants.ROTATE_KP);
    turnPIDController.setI(Constants.ROTATE_KI);
    turnPIDController.setD(Constants.ROTATE_KD);
    turnPIDController.setFF(0.0);
    turnMotor.enableVoltageCompensation(12);
    turnMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
    driveMotor.setSmartCurrentLimit(40);
    //driveMotor.setInverted(false);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveEncoder.setVelocityConversionFactor(Constants.DRIVE_MOTOR_VCONVERSION);
    driveEncoder.setPositionConversionFactor(Constants.DRIVE_MOTOR_PCONVERSION);
    driveController.setP(Constants.ROTATE_KP);
    driveController.setI(Constants.ROTATE_KI);
    driveController.setD(Constants.ROTATE_KD);
    driveController.setFF(0.0);
    driveMotor.enableVoltageCompensation(12);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getPosition());
  }

  public SwerveModuleState getState(){
    double velocity = driveEncoder.getVelocity();
    return new SwerveModuleState(velocity, getAngle());  
  }
}