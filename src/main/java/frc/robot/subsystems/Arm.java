package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Arm extends SubsystemBase{

    private final CANSparkMax raiseMotor1;
    private final CANSparkMax raiseMotor2;
    private final CANSparkMax extendMotor;

    private final RelativeEncoder raiseEncoder1;
    private final RelativeEncoder raiseEncoder2;
    private final RelativeEncoder extendEncoder;

    private boolean isCone;

    private final PIDController armExtendPID;
    private final PIDController armRaisePID1;
    private final PIDController armRaisePID2;

    public Arm(){
        this.raiseMotor1 = new CANSparkMax(13, MotorType.kBrushless);
        this.raiseMotor2 = new CANSparkMax(14, MotorType.kBrushless);
        this.extendMotor = new CANSparkMax(15, MotorType.kBrushless);

        raiseEncoder1 = raiseMotor1.getEncoder();
        raiseEncoder2 = raiseMotor2.getEncoder();
        extendEncoder = extendMotor.getEncoder();

        isCone = false;//Change if initial mode is different

        armExtendPID = new PIDController(1, 0, 0);
        armRaisePID1 = new PIDController(1, 0, 0);
        armRaisePID2 = new PIDController(1, 0, 0);
    }

    public void periodic(){

    }

    public double getRaise1Position(){
        return raiseEncoder1.getPosition();
    }
    public double getRaise2Position(){
        return raiseEncoder2.getPosition();
    }
    public double getExtendPosition(){
        return extendEncoder.getPosition();
    }
    public void setIsCone(boolean isCone){
        this.isCone = isCone;
    }
    public boolean getIsCone(){
        return isCone;
    }
    public void stop(){
        raiseMotor1.stopMotor();
        raiseMotor2.stopMotor();
        extendMotor.stopMotor();
    }

    public void setPreset(String stage){
        if(stage.equals("mid") || stage.equals("high")){
            if(isCone){
                switch(stage){
                    case "mid":
                        extendMotor.set(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get("coneMid")));
                        raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get("coneMid")));
                        raiseMotor2.set(-armRaisePID2.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get("coneMid")));
                        break;
                    case "high":
                        extendMotor.set(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get("coneHigh")));
                        raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get("coneHigh")));
                        raiseMotor2.set(-armRaisePID2.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get("coneHigh")));
                        break;
                }
            }
            else{
                switch(stage){
                    case "mid":
                        extendMotor.set(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get("cubeMid")));
                        raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get("cubeMid")));
                        raiseMotor2.set(-armRaisePID2.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get("cubeMid")));
                        break;
                    case "high":
                        extendMotor.set(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get("cubeHigh")));
                        raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get("cubeHigh")));
                        raiseMotor2.set(-armRaisePID2.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get("cubeHigh")));
                }
            }
        }
        else{
            extendMotor.set(armExtendPID.calculate(getExtendPosition(), OperatorConstants.armExtendPresets.get(stage)));
            raiseMotor1.set(armRaisePID1.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get(stage)));
            raiseMotor2.set(-armRaisePID2.calculate(getRaise1Position(), OperatorConstants.armRaisePresets.get(stage)));
        }
    }
}
