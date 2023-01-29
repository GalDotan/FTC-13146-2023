package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Gripper;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import java.util.ArrayList;

@Autonomous
public class AutoFiveCone extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    public Gripper gripper;
    public Lift lift;
    public Turret turret;

    int active = 1;
    double tile = 24;
    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        turret = new Turret(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        Pose2d end = new Pose2d(36, -55 + tile * 2, Math.toRadians(180));
        Pose2d startPose = new Pose2d(36, -58, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Spline_test = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    gripper.setpose(0);
                })
                .addTemporalMarker(1, () -> {
                    lift.high_pole();
                })
                .splineToConstantHeading(new Vector2d(32.8, -30.6), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(28, -10.3, Math.toRadians(140)), Math.toRadians(140))
                .addDisplacementMarker(() -> {
                    gripper.setpose(1);
                    lift.Stack_5();
                })
                .lineToConstantHeading(new Vector2d(34,-12.4))
                .addDisplacementMarker(() -> {
                    turret.set_encoder(240,-0.6);
                })
                .lineToLinearHeading(new Pose2d(44,-14.4,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(58,-16.7,Math.toRadians(180)))
                .addTemporalMarker(6, ()-> {
                    gripper.setpose(0);
                })
                .addTemporalMarker(7,()-> {
                    lift.high_pole();
                })
                .waitSeconds(5)
                .lineToLinearHeading(new Pose2d(44,-14.4,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(34,-14.4,Math.toRadians(135)))
                .addDisplacementMarker(() -> {
                    turret.set_encoder(0,0.6);
                })
                .splineToLinearHeading(new Pose2d(28.5, -11.3, Math.toRadians(135)), Math.toRadians(140))
                .addDisplacementMarker(() -> {
                    gripper.setpose(1);
                })
                .back(10)
                .turn(Math.toRadians(-45))
                .build();

        TrajectorySequence Right = drive.trajectorySequenceBuilder(Spline_test.end())
                .strafeRight(tile)
                .build();

        TrajectorySequence Left = drive.trajectorySequenceBuilder(Spline_test.end())
                .strafeLeft(tile)
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        while (opModeIsActive() && active ==1) {

            /* Update the telemetry */
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }




            /* Actually do something useful */
            if (tagOfInterest.id == LEFT) {
                telemetry.addLine("Left");
                telemetry.update();
                drive.followTrajectorySequence(Spline_test);
                drive.followTrajectorySequence(Left);
                lift.intake();
                sleep(5000);
                PoseStorage.currentPose = drive.getPoseEstimate();
                telemetry.addLine("Left auto finished");
                telemetry.update();
                active = 0;
            } else if (tagOfInterest.id == MIDDLE) {
                telemetry.addLine("Middle");
                telemetry.update();
                drive.followTrajectorySequence(Spline_test);
                PoseStorage.currentPose = drive.getPoseEstimate();
                lift.intake();
                sleep(5000);
                telemetry.addLine("Middle auto finished");
                telemetry.update();
                active = 0;
            } else if (tagOfInterest.id == RIGHT) {
                telemetry.addLine("Right");
                telemetry.update();
                drive.followTrajectorySequence(Spline_test);
                drive.followTrajectorySequence(Right);
                lift.intake();
                sleep(5000);
                PoseStorage.currentPose = drive.getPoseEstimate();
                telemetry.addLine("Right auto finished");
                telemetry.update();
                active = 0;

            }

        }


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        //telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x));
        //telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        //telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z));
        //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}