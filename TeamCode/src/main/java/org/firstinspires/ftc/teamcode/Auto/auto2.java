package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardwareClass;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardwareClass;
import org.firstinspires.ftc.teamcode.Auto.autoBase;



import java.util.List;



@Autonomous(name="auto2", group = "robot")
public class auto2 extends LinearOpMode {

    hardwareClass robot = new hardwareClass();
    String coneColor = "blank";

    //@Disabled

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /** Wait for the game to begin */
        telemetry.addData(">","Press Play to start op mode");
        telemetry.update();

        encoderDrive(.5, 16, 16, 5);

        //pick up cone
        //robot.onepuHand.setPosition(1);

        senseColor();
        coneColor = senseColor();

        robot.onepuHand.setPosition(1);
        sleep(2000);
        encoderArmUp(1, 6, 5);

        if(coneColor.equals("red")) {
            telemetry.addData("coneColor=", coneColor);
            encoderDrive(.5, 6, 6, 5);
            encoderDrive(.5, -17,17,4);
            encoderDrive(.5, 22, 22, 5);
            encoderDrive(.5, 17,-17,4);
            encoderArmUp(1, 36, 5);
            encoderDrive(.5, 6, 6, 5);
            openHand();
            encoderDrive(.5, -6, -6, 5);
            encoderDrive(.5, 17,-17,4);
            encoderDrive(.5, 4, 4, 5);



           /* robot.fpd.setDirection(DcMotor.Direction.FORWARD);
            robot.bpd.setDirection(DcMotor.Direction.REVERSE);
            robot.fsd.setDirection(DcMotor.Direction.FORWARD);
            robot.bsd.setDirection(DcMotor.Direction.REVERSE);*/
           /* telemetry.addData("i'm trying to strafe", "i'm trying to strafe");
            robot.fpd.setPower(.5);
            robot.bpd.setPower(-.5);
            robot.fsd.setPower(.5);
            robot.bsd.setPower(-.5);

            sleep(1200);
            telemetry.addData("i think that I strafed", "i think that I strafed");
            telemetry.update();*/
        }
        else if(coneColor.equals("blue")) {
            telemetry.addData("coneColor=", coneColor);
            encoderDrive(.5, 6, 6, 5);
        }
        else {
            telemetry.addData("coneColor=", coneColor);
            encoderDrive(.5, 6, 6, 5);

            encoderDrive(.5, 17.5,-17.5,4);

            encoderDrive(.5, 20, 20, 5);

        }

    }

    public void strafeDriveLeft(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorDirStrafeLeft();

        if (opModeIsActive()) {
            newLeftTarget = robot.fpd.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
            newRightTarget = robot.bsd.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);

            telemetry.addData("newLeftTarget", newLeftTarget);
            telemetry.addData("newRightTarget", newRightTarget);
            telemetry.update();

            robot.fpd.setTargetPosition(newLeftTarget);
            robot.bpd.setTargetPosition(newLeftTarget);
            robot.fsd.setTargetPosition(newRightTarget);
            robot.bsd.setTargetPosition(newRightTarget);

            int NT1 = (int) (newLeftTarget * 0.981);
            int NT2 = (int) (newRightTarget * 0.981);

            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.runtime.reset();
            robot.fpd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));

            robot.runtime.reset();
            robot.fpd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));

            while (robot.fpd.isBusy()
                    && robot.bsd.isBusy()
                    && opModeIsActive()
                    && (robot.runtime.seconds() < timeoutS)
                    && ((Math.abs(robot.fpd.getCurrentPosition()) < Math.abs(NT1))
                    && (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2)))
            ) { }

            telemetry.addData("strafeDriveLeft:", "complete");
            telemetry.update();
        }

        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);

        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);
    }

  /*  public void strafeDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        setMotorDirStrafeLeft();

        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.fpd.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
        newRightTarget = robot.bpd.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
        newLeftTarget = robot.fsd.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
        newRightTarget = robot.bsd.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);

        telemetry.addData("newLeftTarget", newLeftTarget);
        telemetry.addData("newRightTarget", newRightTarget);
        telemetry.update();

        robot.fpd.setTargetPosition(newLeftTarget);
        robot.bpd.setTargetPosition(newRightTarget);
        robot.fsd.setTargetPosition(newLeftTarget);
        robot.bsd.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.runtime.reset();
        robot.fpd.setPower(Math.abs(speed));
        robot.bpd.setPower(Math.abs(speed));
        robot.fsd.setPower(Math.abs(speed));
        robot.bsd.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (robot.runtime.seconds() < timeoutS) &&
                (robot.fpd.isBusy() && robot.bpd.isBusy() && robot.fsd.isBusy() && robot.bsd.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Currently at", " at %7d :%7d",
                    robot.fpd.getCurrentPosition(), robot.bpd.getCurrentPosition(), robot.fsd.getCurrentPosition(), robot.bsd.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.fpd.setPower(0);
        robot.bpd.setPower(0);
        robot.fsd.setPower(0);
        robot.bsd.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move.
    } */



    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newfpdTarget = 0;
        int newbpdTarget = 0;
        int newfsdTarget = 0;
        int newbsdTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            telemetry.addData("status:", "starting to drive");

            robot.fpd.setDirection(DcMotor.Direction.FORWARD);
            robot.bpd.setDirection(DcMotor.Direction.FORWARD);
            robot.fsd.setDirection(DcMotor.Direction.REVERSE);
            robot.bsd.setDirection(DcMotor.Direction.REVERSE);

            // Determine new target position, and pass to motor controller
            newfpdTarget = robot.fpd.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
            newbpdTarget = robot.bpd.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
            newfsdTarget = robot.fsd.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
            newbsdTarget = robot.bsd.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);

            telemetry.addData("newfpdTarget", newfpdTarget);
            telemetry.addData("newbpdTarget", newbpdTarget);
            telemetry.addData("newfsdTarget", newfsdTarget);
            telemetry.addData("newbsdTarget", newbsdTarget);
            telemetry.update();

            robot.fpd.setTargetPosition(newfpdTarget);
            robot.bpd.setTargetPosition(newbpdTarget);
            robot.fsd.setTargetPosition(newfsdTarget);
            robot.bsd.setTargetPosition(newbsdTarget);

            // Turn On RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //give the motors a 2% error allowance
            int NT1a = (int) (newfpdTarget * 0.981); //fp
            int NT1b = (int) (newbpdTarget * 0.981); //bp
            int NT2a = (int) (newfsdTarget * 0.981); //fs
            int NT2b = (int) (newbsdTarget * 0.981); //bs

            // reset the timeout time and start motion.
            robot.runtime.reset();
            robot.fpd.setPower(Math.abs(speed));
            robot.bpd.setPower(Math.abs(speed));
            robot.fsd.setPower(Math.abs(speed));
            robot.bsd.setPower(Math.abs(speed));

            while (robot.fpd.isBusy()
                    && robot.bsd.isBusy()
                    && opModeIsActive()
                    && (robot.runtime.seconds() < timeoutS)
                    && ((Math.abs(robot.fpd.getCurrentPosition()) < Math.abs(NT1a))
                    && ((Math.abs(robot.bpd.getCurrentPosition()) < Math.abs(NT1b))
                    && ((Math.abs(robot.fsd.getCurrentPosition()) < Math.abs(NT2a))
                    && (Math.abs(robot.bsd.getCurrentPosition()) < Math.abs(NT2b)))))
            ) { }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.runtime.seconds() < timeoutS) &&
                    (robot.fpd.isBusy() && robot.bpd.isBusy() && robot.fsd.isBusy() && robot.bsd.isBusy()
                    )) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newfpdTarget, newbpdTarget, newfsdTarget, newbsdTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        robot.fpd.getCurrentPosition(), robot.bpd.getCurrentPosition(), robot.fsd.getCurrentPosition(), robot.bsd.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.fpd.setPower(0);
            robot.bpd.setPower(0);
            robot.fsd.setPower(0);
            robot.bsd.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void turn90DegreesClockwise() {
        encoderDrive(1.0, 12, -12, 5); //actual numbers not added
    }

    public void turn90DegreesCounterclockwise() {
        encoderDrive(1.0, -12, 12, 5); //actual numbers not added
    }

    public void turn180DegreesClockwise() {
        encoderDrive(1.0, 24, -24, 5); //actual numbers not added
    }

    public void turn180DegreesCounterclockwise() {
        encoderDrive(1.0, -24, 24, 5); //actual numbers not added
    }

    public void spinClockwise(double power) {
        robot.fpd.setDirection(DcMotor.Direction.FORWARD);
        robot.bpd.setDirection(DcMotor.Direction.FORWARD);
        robot.fsd.setDirection(DcMotor.Direction.FORWARD);
        robot.bsd.setDirection(DcMotor.Direction.FORWARD);

        robot.fpd.setPower(power);
        robot.bpd.setPower(power);
        robot.fsd.setPower(power);
        robot.fpd.setPower(power);

    }

    public void closeHand() {
        robot.onepuHand.setPosition(1);
    }

    public void openHand() {
        robot.onepuHand.setPosition(0);
    }

    public void encoderArmDown(double speed, double inchesUp, double timeoutS){
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

    public void encoderArmUp(double speed, double inchesUp, double timeoutS){
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


    public void spinCounterclockwise(double power) {
        robot.fpd.setDirection(DcMotor.Direction.REVERSE);
        robot.bpd.setDirection(DcMotor.Direction.REVERSE);
        robot.fsd.setDirection(DcMotor.Direction.REVERSE);
        robot.bsd.setDirection(DcMotor.Direction.REVERSE);

        robot.fpd.setPower(power);
        robot.bpd.setPower(power);
        robot.fsd.setPower(power);
        robot.fpd.setPower(power);

    }

    public void getToParking1() {
        encoderDrive(.5, 6, 6, 5);
        //sleep(1000);
        //strafeDrive(.5, 24, 24, 5);
        setMotorDirStrafeLeft();
        setMotorPower(.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1200);  // pause to display final telemetry message.
    }

    public void getToParking2() {
        encoderDrive(.5, 6, 6, 5);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1200);  // pause to display final telemetry message.
    }

    public void getToParking3() {
        encoderDrive(.5, 6, 6, 5);
        //strafeDrive(.5, 24, 24, 5);
        setMotorDirStrafeRight();
        setMotorPower(.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1200);  // pause to display final telemetry message.

    }

    public void setMotorPower(double power) {
        robot.fpd.setPower(power);
        robot.bpd.setPower(power);
        robot.fsd.setPower(power);
        robot.bsd.setPower(power);
    }

    public void testMethod() {
        telemetry.addData("status:", "working");
    }

    public void testMethod2() {
        telemetry.addData("status:", "working 2");
    }

    public void setMotorDirStrafeLeft () {
        robot.fpd.setDirection(DcMotor.Direction.REVERSE);
        robot.bpd.setDirection(DcMotor.Direction.FORWARD);
        robot.fsd.setDirection(DcMotor.Direction.REVERSE);
        robot.bsd.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setMotorDirStrafeRight () {
        robot.fpd.setDirection(DcMotor.Direction.FORWARD);
        robot.bpd.setDirection(DcMotor.Direction.REVERSE);
        robot.fsd.setDirection(DcMotor.Direction.FORWARD);
        robot.bsd.setDirection(DcMotor.Direction.REVERSE);
    }

    public void senseColorTelemetry(){ //drive until see not green
        while (opModeIsActive()){
            telemetry.addData("red", robot.colorSensor.red());
            telemetry.addData("green", robot.colorSensor.green());
            telemetry.addData("blue", robot.colorSensor.blue());


            telemetry.update();

        }
    }

    public String senseColor () {

        String color = "blank";

        while (opModeIsActive() && color.equals("blank")) {

            if (robot.colorSensor.red() > (robot.colorSensor.blue()) && robot.colorSensor.red() > (robot.colorSensor.green())) {
                color = "red";
                telemetry.addData("i see red", " ");
                telemetry.update();
                color = "red";
                //sleeveColor.equals(red);

            } else if (robot.colorSensor.blue() > (robot.colorSensor.red()) && robot.colorSensor.blue() > (robot.colorSensor.green())) {
                color = "blue";
                telemetry.addData("i see blue", " ");
                telemetry.update();
                color = "blue";

            } else if (robot.colorSensor.green() > (robot.colorSensor.red()) && robot.colorSensor.green() > (robot.colorSensor.blue())) {
                color = "green";
                telemetry.addData("i see green", " ");
                telemetry.update();
                color = "green";

            } else {
                telemetry.addData("i see nothing", " ");
                telemetry.update();
                color = "no go";

            }

        }
        return color;
    }



    //start of tensorFlowAuto

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
/*
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
/*    private static final String VUFORIA_KEY =
            "AZnJDqT/////AAABmTzSZmVT4UKrhO3S0lFcbZdpvgH9EVj8mPOzHw3znBryo9aBLeavqVQly0VrGlkp9J9vvYQq2jYkwPCTp83qFymFYA0ktU0uIsIaRDUEGaHTiW70HgoflDYgKFF7XkKcPXs/tuP1bJckNl6RAW8+xDOhvtVlz1rdwrVu5956P1qXoSEl1ulakd+yu8J30VjzM2EJh8tYj0Y9lU8BvWmN18/CF4uqAMdmUxqN9DLR2qYND9mn5Ni0eJuTjxVl/aq8PsCzqca2o0Pd9q5DzorcVcZEHU/QuDVbJIfm4Ti/J1cMP2cp36ZhmceQ7baZOnAhm/152ztS+Qfj6RS4kzPMAqsm4/PznO7ufi109dln8BT/";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
/*    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
/*    private TFObjectDetector tfod;

    //   @Override
    //   public void runOpMode() {
    // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
    // first.
    //       initVuforia();
    //       initTfod();


    /**
     * Activate TensorFlow Object Detection before we wait for the start command.
     * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
     **/




/*

    {
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        String thisLabel = recognition.getLabel();
                        telemetry.addData("this label", thisLabel);
                        telemetry.update();

                        testMethod2();

                        if (thisLabel.equals("2 Bulb")) {
                            testMethod();
                            //encoderDrive(1.0, 40, 40, 40);  // move forward 23.5 inches
                            getToParking2();
                            telemetry.addData("Path", "Complete");
                            telemetry.update();
                            sleep(1000);  // pause to display final telemetry message.

                        }


                       /*     if (thisLabel.equals("1 Bolt")) {
                                telemetry.addData("parking space", "Parking Space 1");
                                telemetry.update();
                                auto.getToParking1();

                            }

                            if(thisLabel.equals("2 Bulb")) {
                                telemetry.addData("parking space", "Parking Space 2");
                                telemetry.update();
                                auto.getToParking2();

                            }

                            if(thisLabel.equals("3 Panel")) {
                                auto.getToParking3();

                            }*/
    //            }


  /*                  telemetry.update();

                }
            }
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */
/*    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
 /*       VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
 /*   private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
*/
}

