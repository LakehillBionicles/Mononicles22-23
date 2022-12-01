package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardwareClass;
import org.firstinspires.ftc.teamcode.Auto.autoBase;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "AprilTagAuto", group = "robot")
public class aprilTagAuto extends LinearOpMode
{
    hardwareClass robot = new hardwareClass();
    autoBase auto = new autoBase();

    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.Auto.aprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

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

        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new aprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //this is where I changed number
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        //telemetry.setMsTransmissionInterval(50);

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
                    //telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                   // tagToTelemetry(aprilTagDetectionPipeline, 0.0);
                }
                else
                {
                    //telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                       // telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                     //   telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        //tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
               // telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                   // telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                   // telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    //tagToTelemetry(tagOfInterest);
                }

            }

         //   telemetry.update();
         //   sleep(20);
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

        /* Actually do something useful */
      /*  if(tagOfInterest == null || tagOfInterest.id == LEFT){
            //code to park left, not tested
            auto.getToParking1();
            auto.spinClockwise(.5);

            //add code to check position?
            // timeout?
        }else if(tagOfInterest.id == MIDDLE){
            //code to park middle, not tested
            auto.getToParking2();
            auto.spinClockwise(.5);

            //add code to check position?
            // timeout?
        }else{
            // code to park right, not tested
            auto.getToParking3();
            auto.spinClockwise(.5);

            //add code to check position?
            // timeout?
        } */


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

  //  public void tagToTelemetry(AprilTagDetection, tagOfInterest) {
  //  }

    // we don't need to mess with this, but we could if we want. would need to fix numbers at top to be accurate
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