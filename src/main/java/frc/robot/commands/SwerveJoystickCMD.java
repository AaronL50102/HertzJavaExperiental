
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCMD extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystickCMD(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction){
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAngularAccelerationUnitsPerSecond);
    addRequirements(swerveSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {
    //Gets real time joystick input
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    
    //Deadband
    xSpeed = Math.abs(xSpeed) > OperatorConstants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.deadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OperatorConstants.deadband ? turningSpeed : 0.0;

    //Limits rate - if pushing drive stick too fast, robot will still accelerate slowly
    //xSpeed = xLimiter.calculate(xSpeed);
    //ySpeed = yLimiter.calculate(ySpeed);
    //turningSpeed = turningLimiter.calculate(turningSpeed);

    ChassisSpeeds chassisSpeeds;
    // if(fieldOrientedFunction.get()){
      // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    // }
    // else{
      // chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    // }

     chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
