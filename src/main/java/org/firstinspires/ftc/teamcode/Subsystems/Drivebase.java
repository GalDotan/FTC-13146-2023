package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.Math;

public class Drivebase{


    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LR = null;
    private DcMotor RR = null;

    public Drivebase(HardwareMap hardwareMap) {
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RR = hardwareMap.get(DcMotor.class, "RR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorPowers(
            double P1,
            double P2,
            double P3,
            double P4
    ) {
        LF.setPower(P1);
        LR.setPower(P2);
        RF.setPower(P3);
        RR.setPower(P4);
    }

    public void vectorDrive(double drive, double strafe, double twist, double flipo) {

        twist = twist*-0.8;
        strafe = strafe*-0.8;
        drive = drive*-0.8;
        if (flipo == 1){
            drive = drive*-1;
            strafe = strafe*-1;
            twist = twist*1;
        }
        LF.setPower(drive - strafe + twist);
        LR.setPower(drive + strafe + twist);
        RF.setPower(-(drive + strafe - twist));
        RR.setPower(-(drive - strafe - twist));

    }

    //private void centricdrive(double drive, double strafe, double twist){
    //
    //    float pi = 3.1415926;
    //
    //    double gyro_degrees = ahrs->GetYaw();
    //    float gyro_radians = gyro_degrees * pi/180;
    //    float temp = drive * Math.cos(gyro_radians) +
    //            strafe * Math.sin(gyro_radians);
    //    strafe = -drive * sin(gyro_radians) +
    //            strafe * Math.cos(gyro_radians);
    //    fwd = temp;
    //}


}
