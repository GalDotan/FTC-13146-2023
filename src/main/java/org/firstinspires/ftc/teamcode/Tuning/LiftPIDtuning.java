package org.firstinspires.ftc.teamcode.Tuning;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


@Config
@Autonomous
public class LiftPIDtuning extends LinearOpMode {


    private DcMotorEx Lift_drive = null;
    public static PIDCoefficients PID = new PIDCoefficients(0,0,0);

    public static double Target_Pose = 1000;
    FtcDashboard dashboard;
    ElapsedTime PIDTimer = new ElapsedTime();
    Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


    @Override
    public void runOpMode(){

        Lift_drive = hardwareMap.get(DcMotorEx.class, "Lift_drive");
        Lift_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
    }







}
