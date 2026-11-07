/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
@Autonomous(name = "Sensor: SparkFun OTOS", group = "Sensor")

public class SensorSparkFunOTOS extends LinearOpMode {
    // Create an instance of the sensor
    SparkFunOTOS myOtos;

    //variables and whatnot
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeMotor = null;
    private boolean toggleIntake = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        //STOP STINKY BAD FIRMWARE FROM OVERRIDING PID THINGY
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        //Wait for start thingy
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        moveWithPID(new Vector2d(10.0, 10.0), 45.0);
    }

    private void doStuffWhileMoving() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        // Reset the tracking if the user requests it
        if (gamepad1.y) {
            myOtos.resetTracking();
        }

        // Re-calibrate the IMU if the user requests it
        if (gamepad1.x) {
            myOtos.calibrateImu();
        }
        // Inform user of available controls
        telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
        telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
        telemetry.addLine();

        // Log the position to the telemetry
        telemetry.addData("X coordinate", pos.x);
        telemetry.addData("Y coordinate", pos.y);
        telemetry.addData("Heading angle", pos.h);
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    private void moveWithPID(Vector2d targetPos, double targetHeading) {
        boolean finished = false;
        double Kp = 0.08;
        double Ki = 0.001;
        double Kd = 0.01;

        double Kp_heading = 0.04;

        //math variables
        Vector2d integralSum = new Vector2d(0.0,0.0);
        Vector2d lastError = new Vector2d(0.0,0.0);
        ElapsedTime timerPID = new ElapsedTime();


        Vector2d errorDist;
        double errorHeading = 0.0;
        //move and stuff
        while (opModeIsActive() && !finished) {
            SparkFunOTOS.Pose2D currentPose = myOtos.getPosition();
            errorDist = new Vector2d(targetPos.x - currentPose.x, targetPos.y - currentPose.y);
            errorHeading = targetHeading - currentPose.h;

            // get time
            double dt = timerPID.seconds();
            if (dt == 0) dt = 0.001; // no divide by 0

            Vector2d derivativeDist = new Vector2d((errorDist.x - lastError.x) / dt, (errorDist.y - lastError.y) / dt);
            integralSum = new Vector2d(integralSum.x + (errorDist.x * dt), integralSum.y + (errorDist.y * dt));

            //math??? something important
            Vector2d drivePower = new Vector2d((Kp * errorDist.x) + (Ki * integralSum.x), (Kp * errorDist.y) + (Ki * integralSum.y));
            double steerCorrection = Kp_heading * errorHeading;
            //more math
            double frontLeftPower  = drivePower.x + drivePower.y + steerCorrection;
            double frontRightPower = drivePower.x - drivePower.y - steerCorrection;
            double backLeftPower   = drivePower.x - drivePower.y + steerCorrection;
            double backRightPower  = drivePower.x + drivePower.y - steerCorrection;

            //clamp it
            double max;
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            //go move do thing drive
            double speedMult = 0.25;
            frontLeftDrive.setPower(frontLeftPower * speedMult);
            frontRightDrive.setPower(frontRightPower * speedMult);
            backLeftDrive.setPower(backLeftPower * speedMult);
            backRightDrive.setPower(backRightPower * speedMult);

            //thingys for next loop
            lastError = errorDist;
            timerPID.reset();

            // Do other stuff not in here
            doStuffWhileMoving();
            telemetry.addData("Target Position", targetPos);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Distance to target position", errorDist);
            telemetry.addData("Distance to target heading", errorHeading);
            telemetry.update();

            // STOP if close enough to place
            if (errorDist.norm() < 0.5) {
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                return;
            }
        }
    }
}
