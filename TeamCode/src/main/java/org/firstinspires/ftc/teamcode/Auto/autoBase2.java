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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardwareClass;
import org.firstinspires.ftc.teamcode.Auto.autoBase;



import java.util.List;
/*
public class autoBase2 extends LinearOpMode{
    hardwareClass robot = new hardwareClass();

    String coneColor = "blank";

    String parkingSpot = "blank";

    @Override
    public void runOpMode(){
        initialize();
    }

    public void initialize(){
        robot.init(hardwareMap);

        robot.fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("ready", "");
        telemetry.update();
    }

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

    public void closeHand() {
        robot.onepuHand.setPosition(1);
    }

    public void openHand() {
        robot.onepuHand.setPosition(0);
    }

    public void encoderArmDown(double speed, double inchesUp, double timeoutS){
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

    public void encoderArmUp(double speed, double inchesUp, double timeoutS){
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

    public void strafeDriveRight(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget = 0;
        int newRightTarget = 0;

        setMotorDirStrafeRight();

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

}
*/