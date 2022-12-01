package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardwareClass;
import org.firstinspires.ftc.teamcode.Auto.autoBase;


@TeleOp(name="onepu_2_driver_tele", group="robot")
//@Disabled
public class onepu_2_driver_tele extends LinearOpMode {
    hardwareClass robot = new hardwareClass();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.initializeMotorDirection();

        waitForStart();

        //Servo onepuHand; //I commented this out Wed night
        //double servoPosition = 0; //I commented this out Wed night

        //onepuHand = hardwareMap.get(Servo.class, "onepuHand"); //I commented this out Wed night
        //onepuHand.setPosition(0);
        //sleep(1000);
        //onepuHand.setPosition(1);
        //sleep(1000);
        //onepuHand.setPosition(0);

        //openHand();

        while (opModeIsActive()) {
            //show the elapsed game time
            telemetry.addData("Status", "Run Time: " + robot.runtime.toString());
            telemetry.update();

            if(gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.left_stick_y > 0.2) {
                while (gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_x > 0.2 || gamepad1.left_stick_y < -0.2 || gamepad1.left_stick_y > 0.2) {

                    drive();
                }
                robot.fpd.setPower(0.0);
                robot.bpd.setPower(0.0);
                robot.fsd.setPower(0.0);
                robot.bsd.setPower(0.0);
            }

            if(gamepad1.right_stick_x < -0.5 || gamepad1.right_stick_x > 0.5 || gamepad1.right_stick_y < -0.5 || gamepad1.right_stick_y > 0.5) {
                while(gamepad1.right_stick_x < -0.5 || gamepad1.right_stick_x > 0.5 || gamepad1.right_stick_y < -0.5 || gamepad1.right_stick_y > 0.5) {
                    spin();
                }
                robot.fpd.setPower(0.0);
                robot.bpd.setPower(0.0);
                robot.fsd.setPower(0.0);
                robot.bsd.setPower(0.0);
            }

            if (gamepad2.left_bumper) {
                telemetry.addData("left Bumper", "left bumper");
                telemetry.update();
                openHand(robot.onepuHand); //changed these because I put it in HW Class, Wed Night
            }

            if(gamepad2.right_bumper) {
                telemetry.addData("right Bumper", "right bumper");
                telemetry.update();
                closeHand(robot.onepuHand); //changed these because I put it in HW Class, Wed Night
            }
            if (gamepad2.left_trigger > 0) {
                robot.onepuArm.setPower(0.3);
            }

            else if (gamepad2.right_trigger > 0) {
                robot.onepuArm.setPower(-0.3);
            }
            else if (gamepad2.right_stick_y!=0){
                robot.onepuArm.setPower(gamepad2.right_stick_y);
            }
            else {
                robot.onepuArm.setPower(0);
            }
            /*if(gamepad1.right_trigger > 0.0 || gamepad1.left_trigger > 0.0) {
                armLift();
            }

            if(gamepad1.y) {
                liftArmByEncoder(1, 33, 5);
                telemetry.addData("arm height: ", "High Post");
                telemetry.update();
            }

            if(gamepad1.x) {
                liftArmByEncoder(1, 24, 5);
                telemetry.addData("arm height: ", "Mid Post");
                telemetry.update();
            }

            if(gamepad1.b) {
                liftArmByEncoder(1, 15, 5);
                telemetry.addData("arm height: ", "Low Post");
                telemetry.update();
            }

            if(gamepad1.a) {
                liftArmByEncoder(1, 4, 5);
                telemetry.addData("arm height: ", "Ground Terminal");
                telemetry.update();
            }*/
        }
    }

    public void drive() {
        if (gamepad1.left_stick_y < -0.3 && (gamepad1.left_stick_x < 0.25 && gamepad1.left_stick_x > -0.25)) {   //forward
            robot.fpd.setPower(-gamepad1.left_stick_y);
            robot.bpd.setPower(-gamepad1.left_stick_y);
            robot.fsd.setPower(-gamepad1.left_stick_y);
            robot.bsd.setPower(-gamepad1.left_stick_y);
            telemetry.addData("for", "");

        } else if (gamepad1.left_stick_y > 0.3 && (gamepad1.left_stick_x > -0.25 && gamepad1.left_stick_x < 0.25)) {   //backwards
            robot.fpd.setPower(-gamepad1.left_stick_y);
            robot.bpd.setPower(-gamepad1.left_stick_y);
            robot.fsd.setPower(-gamepad1.left_stick_y);
            robot.bsd.setPower(-gamepad1.left_stick_y);
            telemetry.addData("back", "");


        } else if (gamepad1.left_stick_x > 0.3 && (gamepad1.left_stick_y > -0.25 && gamepad1.left_stick_y < 0.25)) {  //star


            robot.fsd.setPower(-gamepad1.left_stick_x);
            robot.bsd.setPower(gamepad1.left_stick_x);
            robot.fpd.setPower(gamepad1.left_stick_x);
            robot.bpd.setPower(-gamepad1.left_stick_x);

            telemetry.addData("star", "");

        } else if (gamepad1.left_stick_x < -0.3 && (gamepad1.left_stick_y > -0.25 && gamepad1.left_stick_y < 0.25)) {  //port

            robot.fsd.setPower(-gamepad1.left_stick_x);
            robot.bsd.setPower(gamepad1.left_stick_x);
            robot.fpd.setPower(gamepad1.left_stick_x);
            robot.bpd.setPower(-gamepad1.left_stick_x);

            telemetry.addData("port", "");

        } else {
            //stop

        }

        telemetry.update();

    }

    public void spin() {

        if (gamepad1.right_stick_x > 0.3 && (gamepad1.right_stick_y > -0.5 && gamepad1.right_stick_y < 0.5)) {    //clockwise
            robot.fpd.setPower(gamepad1.right_stick_x);
            robot.bpd.setPower(gamepad1.right_stick_x);
            robot.fsd.setPower(-gamepad1.right_stick_x);
            robot.bsd.setPower(-gamepad1.right_stick_x);


        } else if (gamepad1.right_stick_x < -0.3 && (gamepad1.right_stick_y > -0.5 && gamepad1.right_stick_y < 0.5)) {    //counterclockwise
            robot.fpd.setPower(gamepad1.right_stick_x);
            robot.bpd.setPower(gamepad1.right_stick_x);
            robot.fsd.setPower(-gamepad1.right_stick_x);
            robot.bsd.setPower(-gamepad1.right_stick_x);

        }

    }

    public void armLift () {
        if (gamepad1.left_trigger > 0) {
            robot.onepuArm.setPower(1);
        }

        if (gamepad1.right_trigger > 0) {
            robot.onepuArm.setPower(-1);
        }

        else {
            robot.onepuArm.setPower(0);
        }
    }

    public void openHand(Servo thisHand) {

            Servo onepuHand;
        double servoPosition = 0;

        //thisHand = hardwareMap.get(Servo.class, "onepuHand");
        //onepuHand.setPosition(0);
        //onepuHand.setPosition(0);

        if (gamepad2.left_bumper) {
            thisHand.setPosition(0.2);

        }

    }

    public void closeHand (Servo thisHand) {
        //Servo onepuHand;
        //double servoPosition = 0;

        //onepuHand = hardwareMap.get(Servo.class, "onepuHand");
        //thisHand.setPosition(1);
        if (gamepad2.right_bumper) {
            thisHand.setPosition(1);

        }
    }

    public void lowerArmByEncoder(double speed, double inchesUp, double timeoutS){
        robot.onepuArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newHeightTarget = 0;

        robot.onepuArm.setDirection(DcMotor.Direction.REVERSE);

        if(opModeIsActive()) {
            newHeightTarget = (int) ((inchesUp) * robot.COUNTS_PER_INCH);

            telemetry.addData("newHeightTarget", "newHeightTarget");
            telemetry.update();

            robot.onepuArm.setTargetPosition(newHeightTarget);

            robot.onepuArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int NT1 = (int) (newHeightTarget * 0.981);

            robot.runtime.reset();

            robot.onepuArm.setPower(Math.abs(speed));

            while (robot.onepuArm.isBusy()
                    && opModeIsActive()
                    && (robot.runtime.seconds() < timeoutS)
                    && (Math.abs(robot.onepuArm.getCurrentPosition()) < Math.abs(NT1)))
            { }
        }

        robot.onepuArm.setPower(0);

        robot.onepuArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.onepuArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);
    }

    public void liftArmByEncoder(double speed, double inchesUp, double timeoutS){
        robot.onepuArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newHeightTarget = 0;

        robot.onepuArm.setDirection(DcMotor.Direction.FORWARD);

        if(opModeIsActive()) {
            newHeightTarget = (int) ((inchesUp) * robot.COUNTS_PER_INCH);

            telemetry.addData("newHeightTarget", "newHeightTarget");
            telemetry.update();

            robot.onepuArm.setTargetPosition(newHeightTarget);

            robot.onepuArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int NT1 = (int) (newHeightTarget * 0.981);

            robot.runtime.reset();

            robot.onepuArm.setPower(Math.abs(speed));

            while (robot.onepuArm.isBusy()
                    && opModeIsActive()
                    && (robot.runtime.seconds() < timeoutS)
                    && (Math.abs(robot.onepuArm.getCurrentPosition()) < Math.abs(NT1)))
            { }
        }

        robot.onepuArm.setPower(0);

        robot.onepuArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.onepuArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);
    }





   /* public void turn90Clockwise(){
        if(gamepad1.b !=true){
            auto.turn90DegreesClockwise();
        }
    }

    public void turn90Counterclockwise(){
        if(gamepad1.x !=true){
            auto.turn90DegreesCounterclockwise();
        }
    }

    public void turn180Clockwise(){
        if(gamepad1.y !=true){
            auto.turn180DegreesClockwise();
        }
    }

    public void turn180Counterclockwise(){
        if(gamepad1.b !=true){
            auto.turn180DegreesCounterclockwise();
        }
    } */

}



