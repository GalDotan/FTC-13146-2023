package org.firstinspires.ftc.teamcode.Subsystems;
import java.util.concurrent.TimeUnit;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Teleop;


public class Turret {

    public DcMotorEx Turret = null;
    private DigitalChannel Limit_Back = null;
    private DigitalChannel Limit_Front = null;
    double Last_press = 0;
    boolean FrontLimit_Invert = false;
    boolean BackLimit_Invert = false;


    public Turret(HardwareMap hardwareMap) {
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Limit_Front = hardwareMap.get(DigitalChannel.class, "Limit_Front");
        Limit_Back = hardwareMap.get(DigitalChannel.class, "Limit_Back");
        Limit_Back.setMode((DigitalChannel.Mode.INPUT));
        Limit_Front.setMode((DigitalChannel.Mode.INPUT));
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetMode(DcMotor.RunMode runMode){
        Turret.setMode(runMode);
    }

    public void set_encoder(int ticks,double velo){
        Turret.setTargetPosition(ticks);
        //Turret.setVelocityPIDFCoefficients(13,0,0,1 );
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setPower(velo);
    }

    public void Move_Back(){
        set_encoder(280,800);
    }

    public void Move_Front(){
        set_encoder(0,800);
    }

    public int Get_TurretPose(){
        return(Turret.getCurrentPosition());

    }

    public void Move_Manual(double po){
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setPower(po);
    }

    public boolean Get_FrontLimit(){
        /*
        if (Limit_Front.getState() == true){
            FrontLimit_Invert = false;
        }else if(Limit_Front.getState()==false){
            FrontLimit_Invert = true;
        }
        */
        return !Limit_Front.getState();
    }

    public boolean Get_BackLimit(){
        /*
        if (Limit_Back.getState() == true){
            BackLimit_Invert = false;
        }else if(Limit_Back.getState()==false){
            BackLimit_Invert = true;
        }
        */
        return !Limit_Back.getState();
    }

    public void set_power(double power) {
        Turret.setPower(power);
    }

    public void Move_Toggle(){
        if(Turret.getCurrentPosition() > 110){
            set_encoder(0,800);
            //Last_press = 0;
        }else if(Turret.getCurrentPosition() < 110){
            set_encoder(280,800);
            //Last_press = 1;
        }
    }

}