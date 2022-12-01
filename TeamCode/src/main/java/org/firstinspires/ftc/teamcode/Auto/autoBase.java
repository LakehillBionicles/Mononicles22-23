package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardwareClass;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="autoBase", group = "robot")
public class autoBase extends LinearOpMode {

    hardwareClass robot = new hardwareClass();

    //@Disabled

        @Override
        public void runOpMode() {

            robot.init(hardwareMap);

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Starting at", "%7d :%7d",
                    robot.fpd.getCurrentPosition(),
                    robot.bpd.getCurrentPosition(),
                    robot.fsd.getCurrentPosition(),
                    robot.bsd.getCurrentPosition());
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();


            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
          encoderDrive(.25, 12, 12, 5.0);  // first test, go forward 1 foot



            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // pause to display final telemetry message.
        }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            telemetry.addData("status:", "starting to drive");

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.fpd.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
            newRightTarget = robot.bpd.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
            newLeftTarget = robot.fsd.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
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

        robot. fpd.setPower(power);
        robot. bpd.setPower(power);
        robot. fsd.setPower(power);
        robot. fpd.setPower(power);

    }

    public void spinCounterclockwise(double power) {
        robot. fpd.setDirection(DcMotor.Direction.REVERSE);
        robot. bpd.setDirection(DcMotor.Direction.REVERSE);
        robot.fsd.setDirection(DcMotor.Direction.REVERSE);
        robot. bsd.setDirection(DcMotor.Direction.REVERSE);

        robot. fpd.setPower(power);
        robot.bpd.setPower(power);
        robot.fsd.setPower(power);
        robot.fpd.setPower(power);

    }

    public void getToParking1() {
        turn90DegreesCounterclockwise(); //turn 90 degrees counter clockwise, numbers in hardware class have not been tested, 12in right now, reverse right
        encoderDrive(1.0, 23.5, 23.5, 5); //move forward 23.5 inches
        turn90DegreesClockwise(); //turn 90 degrees clockwise, numbers in hardware class have not been tested, 12in right now, reverse left
        encoderDrive(1.0, 23.5, 23.5, 5); //move forward 23.5 inches


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    public void getToParking2() {
        encoderDrive(1.0, 23.5, 23.5, 5.0);  // move forward 23.5 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    public void getToParking3() {
        turn90DegreesClockwise(); //turn 90 degrees clockwise, numbers in hardware class have not been tested, 12in right now, reverse right
        encoderDrive(1.0, 23.5, 23.5, 5); //move forward 23.5 inches
        turn90DegreesCounterclockwise(); //turn 90 degrees counter clockwise, numbers in hardware class have not been tested, 12in right now, reverse left
        encoderDrive(1.0, 23.5, 23.5, 5); //move forward 23.5 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.

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

}
