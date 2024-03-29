package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class Lift {

    private DcMotorEx Lift_drive = null;
    private static int high_pol_ticks = 4030;
    private static int mid_pol_ticks = 2900;
    private static int low_pol_ticks = 1760;
    private static int intake_hight_ticks = 0;
    private static int front_robot_ticks = 0;
    private static int back_robot_ticks = -280 ;
    private static int min_swivel_hight = -3707;


    public Lift(HardwareMap hardwareMap) {
        Lift_drive = hardwareMap.get(DcMotorEx.class, "Lift_drive");
        Lift_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void set_encoder(int ticks,double power){
        Lift_drive.setTargetPosition(ticks);
        Lift_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift_drive.setPower(-power);
    }

    public void intake(){
        set_encoder(intake_hight_ticks,0.5);
    }

    public void low_pole(){
        set_encoder(low_pol_ticks,-0.9);
    }

    public void mid_pole(){
        set_encoder(mid_pol_ticks,-0.9);
    }

    public void high_pole(){
        set_encoder(high_pol_ticks,-0.9);
    }

    public void high_pole_auto(){
        set_encoder(high_pol_ticks-50,-0.9);
    }

    public int GetPose(){
        return(Lift_drive.getCurrentPosition());

    }

    public void Stack_1(){
        set_encoder(0,-0.4);
    }

    public void Stack_2(){
        set_encoder(110,-0.4);
    }

    public void Stack_3(){
        set_encoder(260,-0.4);
    }

    public void Stack_4(){
        set_encoder(403,-0.4);
    }

    public void Stack_5(){
        set_encoder(567,-0.4);
    }

    public void manual(double po){
        Lift_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift_drive.setPower(po);
    }



}
