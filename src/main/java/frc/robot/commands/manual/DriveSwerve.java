/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manual;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;


public class DriveSwerve extends CommandBase {
  /*
   * Creates a new DriveMecanum.
   */

  private SwerveDrivetrain drivetrain;
  private Supplier<Double>  y, x, z;
  private Supplier<Boolean> fieldTOrientated, toggleX;
  boolean fieldDrive = true, onOff = false;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0);


  public DriveSwerve(SwerveDrivetrain drivetrain, Supplier<Double> yDirect, Supplier<Double> xDirect, 
  Supplier<Double> rotation, Supplier<Boolean> fieldTOrientated, Supplier<Boolean> toggleX) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.toggleX = toggleX;
    this.y = yDirect;
    this.x = xDirect;
    this.z = rotation;
    this.fieldTOrientated = fieldTOrientated; // toggle
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(toggleX.get()){
      onOff = !onOff;
    }
    if(onOff){
      drivetrain.formX();
    }

    /* Get Values, Deadband */
    double translationVal = translationLimiter
        .calculate(MathUtil.applyDeadband(y.get(), Constants.SPEED_DEADBAND));
    double strafeVal = strafeLimiter
        .calculate(MathUtil.applyDeadband(x.get(), Constants.STRAFING_DEADBAND));
    double rotationVal = rotationLimiter
        .calculate(MathUtil.applyDeadband(z.get(), Constants.ROTATION_DEADBAND));

    if(fieldTOrientated.get()){
      fieldDrive = !fieldDrive;
    }

    drivetrain.swerveDrive( new Translation2d(translationVal * 2.5, strafeVal * 2.5),
      rotationVal * 4, fieldDrive, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
    //drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}