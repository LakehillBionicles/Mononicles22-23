package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/*
@Autonomous
public class leftPath extends autoBase2 {
    public void runOpMode() {
        initialize();
        waitForStart();

        telemetry.addData(">","Press Play to start op mode");
        telemetry.update();

        /** Find Parking Spot **/
/*        encoderDrive(.5, 18, 18, 5);
        senseColor();
        coneColor = senseColor();

        if(coneColor.equals("red")) {
            parkingSpot = "1";
        }
        else if(coneColor.equals("blue")) {
            parkingSpot = "2";
        }
        else {
            parkingSpot = "3";
        }

        telemetry.addData("Parking Spot =", parkingSpot);
        telemetry.update();

        /** Score **/
        //pick up cone
/*        robot.onepuHand.setPosition(1);
        //strafe right
        strafeDriveLeft(.5, 36, 36, 5);
        //drive forward to post
        encoderDrive(.5, 6, 6, 5);
        //raise arm
        encoderArmUp(1, 33, 5);
        //release cone
        openHand();
        //lower arm
        encoderArmDown(1, 33.5, 5);
        //back away from post
        encoderDrive(.5, -6, -6, 5);

        /** Park **/
 /*       if (parkingSpot.equals("1")){
            strafeDriveRight(.5, 12, 12, 5);
            encoderDrive(.5, 6, 6, 5);
        }
        else if (parkingSpot.equals("2")){
            strafeDriveRight(.5, 48, 48, 5);
            encoderDrive(.5, 6, 6, 5);
        }
        else {
            strafeDriveRight(.5, 60, 60, 5);
            encoderDrive(.5, 6, 6, 5);
        }

        stop();
    }

} */
