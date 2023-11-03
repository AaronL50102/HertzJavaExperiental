package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDController turningPidController;

    private double percentSpeed;

    private final CANCoder absoluteEncoder;

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderID, double TurnkP, double TurnkI, double TurnkD){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Meter);
        turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2MeterPerSec);

        absoluteEncoder = new CANCoder(absoluteEncoderID);

        turningPidController = new PIDController(TurnkP, TurnkI, TurnkD);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        percentSpeed = 1;

        resetTurnEncoders();
        resetDriveEncoders();
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurnPosition(){
        return turnEncoder.getPosition();
    }
    public double getTurnVelocity(){
        return turnEncoder.getVelocity();
    }
    public double getAbsoluteEncoderAngle(){
        //return absoluteEncoder.getBusVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI;
        return absoluteEncoder.getAbsolutePosition() * 2 * Math.PI / 360;
    }

    public void resetTurnEncoders(){
        turnEncoder.setPosition(0);
    }
    public void resetDriveEncoders(){
        driveEncoder.setPosition(0);
    }

    public void setPercentSpeed(double speed){
        this.percentSpeed = speed;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }
    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            driveMotor.set(0);
            turnMotor.set(0);
            return;
        }
        // state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond  /*/ DriveConstants.physicalMaxSpeedMetersPerSecond*/);
        turnMotor.set(turningPidController.calculate(getTurnPosition(), state.angle.getRadians()));        
    }
}
