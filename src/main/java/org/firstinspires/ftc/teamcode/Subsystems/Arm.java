package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Mat;

import java.lang.Math;

public class Arm {

    private DcMotorEx Arm_base = null;
    private DcMotorEx Arm_joint = null;

    private final double b = 250.5; // Arm l

    double x = 0;
    double y = 0;



    public Arm(HardwareMap hardwareMap){
        Arm_base = hardwareMap.get(DcMotorEx.class, "Arm_base");
        Arm_joint = hardwareMap.get(DcMotorEx.class, "Arm_joint");
        Arm_base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset_motors(){
        Arm_base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_joint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void cal_XY(boolean x_factor, boolean y_factor){
        if (x_factor){
            x = x + 1;
        } else if(y_factor) {
            y = y + 1;
        }
    }

    public void move_to_pose(double x, double y, double z){


        double l = Math.sqrt(x*x+y*y);
        double h = Math.sqrt(l*l+z*z);
        double phi = Math.atan(z/l) * (180/Math.PI);
        double theta = Math.acos((h/2)/b) * (180/Math.PI);

        double a1 = phi + theta;
        double a2 = phi - theta;

        int a1_round = (int)Math.round(a1);
        int a2_round = (int)Math.round(a2);

        Arm_base.setTargetPosition(a1_round);
        Arm_joint.setTargetPosition(a2_round);
    }










}
