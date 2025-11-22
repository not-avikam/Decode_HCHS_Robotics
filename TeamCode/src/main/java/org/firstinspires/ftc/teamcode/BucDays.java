package org.firstinspires.ftc.teamcode;/*
import android.util.Size;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name="Cocked", group="Buc Days")
public class BucDays extends LinearOpMode {
    PIDFController pidf = new PIDFController(0, 0, 0, 0);
    private final ElapsedTime runtime = new ElapsedTime();
    private final Pose startPose = new Pose(135, 133, Math.toRadians(0));
    private final Pose observationZone = new Pose(120, 120);
    private final Pose basket = new Pose(120, 25);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setTargetColorRange(ColorRange.YELLOW)// use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
        follower.setStartingPose(startPose);
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312)
        );
        MotorEx vSlideLeft = new MotorEx(hardwareMap, "VSL", Motor.GoBILDA.RPM_435);
        MotorEx vSlideRight = new MotorEx(hardwareMap, "VSR", Motor.GoBILDA.RPM_435);
        MotorGroup vSlides = new MotorGroup(vSlideLeft, vSlideRight);
        CRServo linSlideLeft = hardwareMap.get(CRServo.class, "LSL");
        CRServo linSlideRight = hardwareMap.get(CRServo.class, "LSR");
        ServoImplEx claw = hardwareMap.get(ServoImplEx.class, "claw");
        ServoImplEx clawAdjust = hardwareMap.get(ServoImplEx.class, "cA");
        ServoEx clawRotateLeft = new SimpleServo(hardwareMap, "cRL", 0, 300, AngleUnit.DEGREES);
        ServoEx clawRotateRight = new SimpleServo(hardwareMap, "cRR", 0, 300, AngleUnit.DEGREES);
        ServoEx intakeRotate = new SimpleServo(hardwareMap, "iR", 0, 300, AngleUnit.DEGREES);
        ServoEx intakeWrist = new SimpleServo(hardwareMap, "iC", 0, 180, AngleUnit.DEGREES);
        ServoEx intakeClaw = new SimpleServo(hardwareMap, "iC", 0, 180, AngleUnit.DEGREES);
        //BNO055IMUNew imu = hardwareMap.get(BNO055IMUNew.class, "imu");

        vSlides.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        linSlideLeft.setDirection(CRServo.Direction.REVERSE);
        clawRotateLeft.setInverted(true);
        clawRotateRight.setInverted(true);
        claw.setDirection(ServoImplEx.Direction.REVERSE);
        vSlideRight.setInverted(true);

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

       // telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        follower.startTeleopDrive();
        follower.setStartingPose(startPose);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("preview on/off", "... Camera Stream\n");
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);// filter out very small blobs.

            double slow_forwardbackward = (-gamepad1.left_stick_y*.5);
            double slow_strafe = (-gamepad1.left_stick_x*.5);
            double slow_rotate = (-gamepad1.right_stick_x*.5);

            if (gamepad1.right_bumper) {
                follower.setTeleOpMovementVectors(slow_forwardbackward, slow_strafe, slow_rotate, false);
                follower.update();
            } else {
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
                follower.update();
            }

            if (gamepad1.right_trigger != 0) {
                linSlideLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                linSlideRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            } else if (gamepad1.left_trigger != 0) {
                linSlideLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                linSlideRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            } else if (follower.getVelocityMagnitude() >= 1.5){
                linSlideLeft.setPower(-1 * ((Math.abs((Math.pow(follower.getVelocityMagnitude(), 2) * .55 / .2)/1.79)/2)/100));
                linSlideRight.setPower(-1 * ((Math.abs((Math.pow(follower.getVelocityMagnitude(), 2) * .55 / .2)/1.79)/2)/100));
            } else {
                linSlideLeft.setPower(0);
                linSlideRight.setPower(0);
            }

            if (gamepad2.dpad_up) {
                clawRotateLeft.setPosition(0.555);
                clawRotateRight.setPosition(.833);
                clawAdjust.setPosition(.75);
                telemetry.addLine("Sample scoring");
                intakeWrist.setPosition(1);
            } else if (gamepad2.dpad_down) {
                clawRotateLeft.setPosition(0);
                clawRotateRight.setPosition(0);
                clawAdjust.setPosition(.12 - .0277);
                intakeWrist.setPosition(0);
                telemetry.addLine("Resetting claw to intake");
            } else if (gamepad2.dpad_left) {
                clawRotateLeft.setPosition(.06);
                clawRotateRight.setPosition(.09);
                clawAdjust.setPosition(0.5);
                telemetry.addLine("Specimen pickup");
            }

            if (gamepad2.right_trigger != 0) {
                vSlides.set(gamepad2.right_trigger - gamepad2.left_trigger);
            } else if (gamepad2.left_trigger != 0) {
                vSlides.set(gamepad2.right_trigger - gamepad2.left_trigger);
            } else if (gamepad2.right_trigger == 0) {
                vSlides.set(0);
            } else if (gamepad2.left_trigger == 0) {
                vSlides.set(0);
            }


            if (gamepad2.y) {
                claw.setPosition(1);
            } else if (gamepad2.b) {
                claw.setPosition(0);
            }

            org.opencv.core.Size myBoxFitSize;
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                myBoxFitSize = boxFit.size;
                telemetry.addData("width", myBoxFitSize.width);
                telemetry.addData("height", myBoxFitSize.height);
                telemetry.addData("angle", boxFit.angle);
                intakeRotate.turnToAngle(boxFit.angle);
            }

            if (gamepad1.x) {
                intakeClaw.setPosition(1);
            }  else if (gamepad1.a){
                intakeClaw.setPosition(0);
            }

            if (gamepad2.right_stick_button) {
                vSlides.setRunMode(Motor.RunMode.RawPower);
                vSlides.set(gamepad2.right_trigger - gamepad2.left_trigger);
                linSlideLeft.setPower(-1 * ((Math.abs(6/1.79)/2)/100));
                linSlideRight.setPower(-1 * ((Math.abs(6/1.79)/2)/100));
                claw.setPwmDisable();
                clawRotateLeft.disable();
                clawRotateRight.disable();
                clawAdjust.setPwmDisable();
                intakeClaw.disable();
                intakeRotate.disable();
                intakeWrist.disable();
                telemetry.addLine("Hang mode");
            }

            //telemetry.addData("Vslides position", "%.2f", vSlides.getCurrentPosition());
            //telemetry.addData("Vslides distance", "%.2f", vSlides.getDistance());
            telemetry.addLine(" Area Density Aspect  Center");
            telemetry.update();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("preview on/off", "... Camera Stream\n");
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
            follower.update();
        }
    }
}

 */