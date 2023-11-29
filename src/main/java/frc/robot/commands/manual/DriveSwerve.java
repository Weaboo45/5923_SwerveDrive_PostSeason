/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manual;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

import static frc.robot.Constants.*;

public class DriveSwerve extends CommandBase {
  /*
   * Creates a new DriveMecanum.
   */

  private SwerveDrivetrain drivetrain;
  private Supplier<Double>  y, x, z;
  private Supplier<Boolean> fieldTOrientated;
  boolean fieldDrive = true;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(4.0);

  public DriveSwerve(SwerveDrivetrain drivetrain, 
  Supplier<Double> yDirect, Supplier<Double> xDirect, 
  Supplier<Double> rotation, 
    Supplier<Boolean> fieldTOrientated) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
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
    double yDirect = -y.get();
    double xDirect = x.get();
    double rotation = z.get();
    fieldDrive = fieldTOrientated.get();

    yDirect = yDirect > 0.5 ? yDirect * 0.20 : yDirect;
    xDirect = xDirect > 0.5 ? xDirect * 0.20 : xDirect;
    rotation = rotation > 0.5 ? rotation * 0.20 : rotation;

    yDirect = yLimiter.calculate(yDirect);
    xDirect = xLimiter.calculate(xDirect);
    rotation = turnLimiter.calculate(rotation);

    drivetrain.swerveDrive(yDirect, xDirect, rotation, fieldDrive, new Translation2d()); //yDirect, xDirect
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}