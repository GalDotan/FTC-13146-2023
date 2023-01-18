/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
public class AutoTwoCone extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    double tile = 20;
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
        gripper.setpose(0);

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

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        Pose2d end = new Pose2d(36,-55+tile*2,Math.toRadians(180));
        Pose2d startPose = new Pose2d(36, -55,Math.toRadians(90) );
        drive.setPoseEstimate(startPose);

        TrajectorySequence Spline_test = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(end,Math.toRadians(0))
                //.turn(Math.toRadians(90))
                //.back(tile)
                //.splineTo(new Vector2d(24,0),Math.toRadians(0))
                .build();



        TrajectorySequence Tg1 = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(20,() -> {
                    lift.high_pole();
                })
                .forward(tile*2)
                .turn(Math.toRadians(-45))
                .forward(15)
                .back(3)
                .addDisplacementMarker(() -> {
                    gripper.setpose(0);
                    //lift.high_pole();
                })
                .build();

        TrajectorySequence Tg2 = drive.trajectorySequenceBuilder(Tg1.end())
                .back(15)
                .turn(Math.toRadians(135))
                .forward(tile)
                .build();

        TrajectorySequence Tg3 = drive.trajectorySequenceBuilder(Tg2.end())
                .back(tile)
                .turn(Math.toRadians(-135))
                .forward(15)
                .build();

        TrajectorySequence Tg4 = drive.trajectorySequenceBuilder(Tg3.end())
                .back(15)
                .turn(Math.toRadians(135))
                .forward(tile)
                .build();

        TrajectorySequence Tg5 = drive.trajectorySequenceBuilder(Tg4.end())
                .back(tile)
                .turn(Math.toRadians(-135))
                .forward(15)
                .build();

        TrajectorySequence Left = drive.trajectorySequenceBuilder(Tg5.end())
                .back(15)
                .turn((Math.toRadians(45)))
                .back(10)
                .strafeLeft(tile)
                .build();

        TrajectorySequence Middle = drive.trajectorySequenceBuilder(Tg5.end())
                .back(15)
                .turn((Math.toRadians(45)))
                .back(10)
                .build();

        TrajectorySequence Right = drive.trajectorySequenceBuilder(Tg5.end())
                .back(15)
                .turn((Math.toRadians(45)))
                .back(10)
                .strafeRight(tile)
                .build();



        /* Actually do something useful */
        if(tagOfInterest.id == LEFT){
            telemetry.addLine("Left");
            telemetry.update();
            //drive.followTrajectorySequence(Tg1);
            //move lift up
            //drop cone
            //drive.followTrajectorySequence(Tg2);
            //lower lift
            //pickup cone
            //drive.followTrajectorySequence(Tg3);
            //move lift up
            //drop cone
            //drive.followTrajectorySequence(Tg4);
            //lower lift
            //pickup cone
            //drive.followTrajectorySequence(Tg5);
            //move lift up
            //drop cone
            //drive.followTrajectorySequence(Left);
            drive.followTrajectorySequence(Spline_test);
        }else if(tagOfInterest.id == MIDDLE){
            telemetry.addLine("Middle");
            telemetry.update();
            gripper.setpose(0);
            drive.followTrajectorySequence(Tg1);
            //move lift up
            //drop cone
            gripper.setpose(1);
            drive.followTrajectorySequence(Tg2);
            //lower lift
            //pickup cone
            drive.followTrajectorySequence(Tg3);
            //move lift up
            //drop cone
            drive.followTrajectorySequence(Tg4);
            //lower lift
            //pickup cone
            drive.followTrajectorySequence(Tg5);
            //move lift up
            //drop cone
            drive.followTrajectorySequence(Middle);
        }else if(tagOfInterest.id == RIGHT){
            telemetry.addLine("Right");
            telemetry.update();
            drive.followTrajectorySequence(Tg1);
            //move lift up
            //drop cone
            drive.followTrajectorySequence(Tg2);
            //lower lift
            //pickup cone
            drive.followTrajectorySequence(Tg3);
            //move lift up
            //drop cone
            drive.followTrajectorySequence(Tg4);
            //lower lift
            //pickup cone
            drive.followTrajectorySequence(Tg5);
            //move lift up
            //drop cone
            drive.followTrajectorySequence(Right);
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}