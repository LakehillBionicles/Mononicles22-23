package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.ArrayList;
import java.util.List;

public class hardwareClass extends LinearOpMode {
    // drive train motors
    public DcMotor fpd = null;
    public DcMotor bpd = null;
    public DcMotor fsd = null;
    public DcMotor bsd = null;

    public Servo onepuHand = null;

    public DcMotor onepuArm = null; // not sure this is actually a dc motor

    public ColorSensor colorSensor = null;

    public ElapsedTime runtime = new ElapsedTime();




    // can't calculate counts per inch yet
    public static final double COUNTS_PER_MOTOR_REV = 560;    // try multiplying by 4 and 1.2
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    public static final double WHEEL_DIAMETER_INCHES = 3.9;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    HardwareMap hwMap = null;
    public ElapsedTime time = new ElapsedTime();

    /* Constructor */
    public hardwareClass() {
    }

    //@Override

    public void runOpMode() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        fpd = hwMap.get(DcMotor.class, "fpd");
        bpd = hwMap.get(DcMotor.class, "bpd");
        fsd = hwMap.get(DcMotor.class, "fsd");
        bsd = hwMap.get(DcMotor.class, "bsd");

        onepuHand = hwMap.get(Servo.class, "onepuHand");

        onepuArm = hwMap.get(DcMotor.class, "onepuArm");

        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        fpd.setDirection(DcMotor.Direction.FORWARD);
        bpd.setDirection(DcMotor.Direction.FORWARD);
        fsd.setDirection(DcMotor.Direction.REVERSE);
        bsd.setDirection(DcMotor.Direction.REVERSE);

     /*   fpd.setPower(0);
        bpd.setPower(0);
        fsd.setPower(0);
        bsd.setPower(0);

        fpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bpd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bsd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bpd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bsd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); */

    }



    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
 /*   public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = fpd.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = bpd.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarget = fsd.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = bsd.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            telemetry.addData("newLeftTarget", newLeftTarget);
            telemetry.addData("newRightTarget", newRightTarget);
            telemetry.update();

            fpd.setTargetPosition(newLeftTarget);
            bpd.setTargetPosition(newRightTarget);
            fsd.setTargetPosition(newLeftTarget);
            fpd.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fsd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fpd.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fpd.setPower(Math.abs(speed));
            bpd.setPower(Math.abs(speed));
            fsd.setPower(Math.abs(speed));
            bsd.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fpd.isBusy() && bpd.isBusy() && fsd.isBusy() && bsd.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        fpd.getCurrentPosition(), bpd.getCurrentPosition(), fsd.getCurrentPosition(), bsd.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fpd.setPower(0);
            bpd.setPower(0);
            fsd.setPower(0);
            bsd.setPower(0);

            // Turn off RUN_TO_POSITION
            fpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bpd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bsd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    } */


    //methods I've added

    public void initializeMotorDirection() {
        fpd.setDirection(DcMotor.Direction.FORWARD);
        bpd.setDirection(DcMotor.Direction.FORWARD);
        fsd.setDirection(DcMotor.Direction.REVERSE);
        bsd.setDirection(DcMotor.Direction.REVERSE);
    }

   /* public void turn90DegreesClockwise() {
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
        fpd.setDirection(DcMotor.Direction.FORWARD);
        bpd.setDirection(DcMotor.Direction.FORWARD);
        fsd.setDirection(DcMotor.Direction.FORWARD);
        bsd.setDirection(DcMotor.Direction.FORWARD);

        fpd.setPower(power);
        bpd.setPower(power);
        fsd.setPower(power);
        fpd.setPower(power);

    }

    public void spinCounterclockwise(double power) {
        fpd.setDirection(DcMotor.Direction.REVERSE);
        bpd.setDirection(DcMotor.Direction.REVERSE);
        fsd.setDirection(DcMotor.Direction.REVERSE);
        bsd.setDirection(DcMotor.Direction.REVERSE);

        fpd.setPower(power);
        bpd.setPower(power);
        fsd.setPower(power);
        fpd.setPower(power);

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
        fpd.setPower(power);
        bpd.setPower(power);
        fsd.setPower(power);
        bsd.setPower(power);
    }
*/
}
    /*
    Also add:
    - turn 180
     */