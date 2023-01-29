package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Auto.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.Locale;
import java.util.Map;
import java.util.Vector;


@Config
@TeleOp(name = "Teleop", group = "main")
public class Teleop extends LinearOpMode {



    public Gripper gripper;
    public Lift lift;
    public Turret turret;
    double turret_back = 0;
    double turret_front = 0;
    double flip = 0;



    @Override
    public void runOpMode() {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(36, -58, Math.toRadians(0)));
        waitForStart();

        while (opModeIsActive()) {

           //Drive train drive function
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y*0.6,
                    -gamepad1.left_stick_x*0.6
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                           -gamepad1.right_stick_x*0.5
                    )
            );
            drive.update();

            //Gripper position
            gripper.setpose(-gamepad2.left_stick_y);

            if (gamepad1.a){
                lift.Stack_5();
            }else if (gamepad1.b){
                lift.Stack_4();
            }else if (gamepad1.y){
                lift.Stack_3();
            } else if (gamepad1.x) {
                lift.Stack_2();
            }



            //Systems gamepad functions
            if (gamepad2.a){
                lift.intake();
            }else if (gamepad2.x){
                lift.low_pole();
            }else if (gamepad2.y){
                lift.mid_pole();
            }else if (gamepad2.b){
                lift.high_pole();
            }else if (gamepad2.left_bumper){
                if (turret.Get_TurretPose() > 120){
                    turret_back = 1;
                }else{
                    turret_front = 1;
                }
            }else if (gamepad2.right_bumper){
                if (turret.Get_TurretPose() > 120){
                    turret_front = 1;
                }else{
                    turret_back = 1;
                }
            }

            if (turret_front == 0 && turret_back == 0){
                turret.set_power(gamepad2.right_stick_y*0.5);
            }

            telemetry.addData("Turret fron value",turret_front);
            if (turret_front == 1){
                if (turret.Get_BackLimit() == true){
                    turret.set_power(0);
                    turret_front = 0;
                }else{
                    turret.set_power(0.47);
                }
            }

            if (turret_back == 1){
                if (turret.Get_FrontLimit() == true){
                    turret.set_power(0);
                    turret_back = 0;
                }else{
                    turret.set_power(-0.47);
                }
            }


            //prints-Manual control commented
            //lift.manual(gamepad2.right_stick_y);
            telemetry.addData("X", gamepad2.right_trigger);
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("Heading", poseEstimate.getHeading());
            telemetry.addData("Lift", lift.GetPose());
            telemetry.update();


        }
    }

}