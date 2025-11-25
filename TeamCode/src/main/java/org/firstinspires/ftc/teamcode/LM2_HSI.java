/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */


@TeleOp(name = "Decode LM2", group = "HSI LM2")
//@Disabled
public class LM2_HSI extends OpMode {
    private Servo pitch = null;
    private Servo yaw = null;

    private DcMotorEx launcher = null;
    private Servo agigtator = null;
    private Servo indexer = null;
    private Pose scoreFace = new Pose(137,142);
    private Follower follower;
    public Pose startingPose; //See ExampleAuto to understand how to use this
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Declare OpMode members.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        agigtator = hardwareMap.get(Servo.class, "agigtator");
        pitch = hardwareMap.get(Servo.class, "pitch");
        indexer = hardwareMap.get(Servo.class, "indexer");
        yaw = hardwareMap.get(Servo.class, "yaw");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        agigtator.setPosition(0);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(.05, .5, .05, .6));

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addLine("Press right on the dpad for red alliance");
        telemetry.addLine("Press left on the dpad for blue alliance");
        if (gamepad1.dpad_right) {
            scoreFace = new Pose(137, 142);
        } else if (gamepad1.dpad_left) {
            scoreFace = new Pose(7, 142);
        }
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        if (gamepad1.aWasPressed()) {
            slowMode = true;
            slowModeMultiplier = 0.5;
        } else if (gamepad1.aWasReleased()) {
            slowMode = false;
            slowModeMultiplier = 1;
        }

        if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false // Robot Centric
        );
            //This is how it looks with slowMode on
        else if (slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowModeMultiplier,
                -gamepad1.left_stick_x * slowModeMultiplier,
                -gamepad1.right_stick_x * slowModeMultiplier,
                false // Robot Centric
        );

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

        if (launcher.getVelocity() == 2300) {
            gamepad1.rumble(1000);
        }

        if (gamepad1.left_trigger != 0) {
            aim();
        } else {
            launcher.setPower(0);
        }

        if (gamepad1.right_trigger !=0) {
            agigtator.setPosition(.3);
        } else {
            agigtator.setPosition(0);
        }

        if (gamepad1.dpadRightWasPressed() && agigtator.getPosition() == 0) {
            indexer.setPosition(indexer.getPosition()+.333);
        } else if (gamepad1.dpadLeftWasPressed() && agigtator.getPosition() == 0) {
            indexer.setPosition(indexer.getPosition()-.333);
        }
    }

    public void aim() {

        PathChain score = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), follower.getPose()))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(scoreFace))
                .build();

        double distanceToTarget = Math.sqrt(
                Math.pow(follower.getPose().getX() - 137, 2) +
                        Math.pow(follower.getPose().getY() - 142, 2)
        );

        double launchAngle = Math.toDegrees(Math.atan2(54, distanceToTarget));

        // 1. Define goal and get robot pose
        // 1. Define goal and get robot pose
        double goalX = 137;
        double goalY = 142;
        // 2. Calculate the difference (vector) from robot to goal
        double deltaX = goalX - follower.getPose().getX();
        double deltaY = goalY - follower.getPose().getY();

        // 3. Calculate the absolute field-centric angle to the goal
        double absoluteAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // 4. Calculate the relative angle for the turret by subtracting the robot's heading
        double yawAngle = absoluteAngleToGoal - Math.toDegrees(follower.getHeading());

        follower.followPath(score);

        yaw.setPosition(yawAngle/1800);

        telemetry.addData("Launch Angle", launchAngle);

        pitch.setPosition(launchAngle/300);

        launcher.setPower(1);
        follower.update();
        telemetry.addData("Launch Angle", launchAngle);
    }
}


