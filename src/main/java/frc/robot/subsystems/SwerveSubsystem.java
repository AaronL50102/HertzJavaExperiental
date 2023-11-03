package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveJoystickCMD;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.frontLeftDriveMotorPort,
        DriveConstants.frontLeftTurningMotorPort,
        false, 
        false,
        DriveConstants.frontLeftAbsoluteEncoder, 0.2, 0, 0);
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.frontRightDriveMotorPort,
        DriveConstants.frontRightTurningMotorPort,
        false,
        false,
        DriveConstants.frontRightAbsoluteEncoder, 0.2, 0, 0);
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.backLeftDriveMotorPort,
        DriveConstants.backLeftTurningMotorPort,
        false,
        false,
        DriveConstants.backLeftAbsoluteEncoder, 0.2, 0, 0);
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.backRightDriveMotorPort,
        DriveConstants.backRightTurningMotorPort,
        false,
        false,
        DriveConstants.backRightAbsoluteEncoder, 0.2, 0, 0);

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB1);

    private int count = 0;

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.SwerveDriveKinematics, new Rotation2d(0));
    

    public SwerveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
            } catch(Exception e){}
            zeroHeading();
        }).start();
    }

    public SwerveModule getFL(){
        return frontLeft;
    }
    public SwerveModule getFR(){
        return frontRight;
    }
    public SwerveModule getBL(){
        return backLeft;
    }
    public SwerveModule getBR(){
        return backRight;
    }

    public void zeroHeading(){
        gyro.reset();
    }
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void periodic(){
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        // count++;
        // if(count >= 100){
        //     resetModules();
        //     count = 0;
        // }
        SmartDashboard.putNumber("count", count);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Front Left", frontLeft.getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Front Right", frontRight.getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Back Left", backLeft.getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Back Right", backRight.getAbsoluteEncoderAngle());

        SmartDashboard.putNumber("Front Left2", frontLeft.getTurnPosition());
        SmartDashboard.putNumber("Front Right2", frontRight.getTurnPosition());
        SmartDashboard.putNumber("Back Left2", backLeft.getTurnPosition());
        SmartDashboard.putNumber("Back Right2", backRight.getTurnPosition());
    }
    
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }

    public void resetOdometer(Pose2d pose){
        odometer.resetPosition(pose, getRotation2d())
    }

    public void resetModules(){
        frontLeft.resetTurnEncoders();
        frontRight.resetTurnEncoders();
        backLeft.resetTurnEncoders();
        backRight.resetTurnEncoders();
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setHalfSpeed(double speed){
        frontLeft.setPercentSpeed(speed);
        frontRight.setPercentSpeed(speed);
        backLeft.setPercentSpeed(speed);
        backRight.setPercentSpeed(speed);
    }
    
    public void setModuleStates(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
        SmartDashboard.putNumber("angle 1", states[0].angle.getRadians());
        SmartDashboard.putNumber("angle 2", states[1].angle.getRadians());
        SmartDashboard.putNumber("angle 3", states[2].angle.getRadians());
        SmartDashboard.putNumber("angle 4", states[3].angle.getRadians());
    }
}
