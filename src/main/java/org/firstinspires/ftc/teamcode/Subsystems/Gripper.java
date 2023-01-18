package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    Servo Gripper = null;


    public Gripper(HardwareMap hardwareMap){

        Gripper = hardwareMap.servo.get("Gripper");
    }

    public void idle(){
        Gripper.setPosition(0.5);
    }

    public void setpose(double position){
        Gripper.setPosition(position);
    }

    public double get_pose(){
        return(Gripper.getPosition());
    }


}
