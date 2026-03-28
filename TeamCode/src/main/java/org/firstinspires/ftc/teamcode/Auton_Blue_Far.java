package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

@Autonomous(name="Far Blue", group="lcq")
public class Auton_Blue_Far extends OpMode {
    private Servo light;
    ColorBlobLocatorProcessor colorLocator;
    private MotorEx launcher = null;
    InterpLUT lut;

    private Servo agitator;
    double targetVelocity = 1800;
    double realVelocity = 1500;
    private Motor intake = null;
    private Servo trigger = null;
    private ServoEx pitch = null;
    NormalizedColorSensor colorSensor;
    private final Pose startPose = new Pose(58.745, 11.244, Math.toRadians(109));
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;
    private int pathState;
    private int shootBall;
    private PathChain score1, pickup1, score2, pickup2, score3, pickup3;
    private final Pose scorePose = new Pose(11, 136);
    double targetDistance;
    public void buildPaths() {

        pickup1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.745, 11.244),
                                new Pose(58.745, 39.095),
                                new Pose(12.326, 35.501)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(200))

                .build();

        score1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.326, 35.501),

                                new Pose(56.128, 12.270)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(109))

                .build();

        pickup2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.128, 12.270),
                                new Pose(62.566, 70.101),
                                new Pose(12.588, 55.811)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(180))

                .build();

        score2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(12.588, 55.811),
                                new Pose(45.640, 68.262),
                                new Pose(53.227, 80.702)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(128))

                .build();

        pickup3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(53.227, 80.702),
                                new Pose(49.948, 83.531),
                                new Pose(19.716, 84.853)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(128), Math.toRadians(190))

                .build();

        score3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.716, 84.853),

                                new Pose(44.969, 92.756)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(190), Math.toRadians(128))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                targetVelocity = 2200;
                realVelocity = 2000;
                shoot();
                if (shootBall >= 2) {
                    follower.followPath(pickup1, false);
                    setPathState(1);
                }
                break;
            case 1:
                intake.set(1);
                if(!follower.isBusy()) {
                    intake.set(0);
                    follower.followPath(score1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    targetVelocity = 1750;
                    realVelocity = 1500;
                    shoot();
                    if (shootBall >= 2){
                        follower.followPath(pickup2, false);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                intake.set(1);
                if(!follower.isBusy()) {
                    intake.set(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    shoot();
                    if (shootBall >= 2) {
                        follower.followPath(pickup3, false);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                intake.set(1);
                if(!follower.isBusy()) {
                    follower.followPath(score3, true);
                    setPathState(8);
                    intake.set(0);
                }
                break;
            case 8:
                setShootBall(0);
                if(!follower.isBusy()) {
                    shoot();
                    if (shootBall >= 2) {
                        telemetry.addLine("Autonomous Routine Completed");
                        telemetry.addLine("Good Luck!");
                        setPathState(9);
                    }
                }
                break;
                /*
            case 9:
                Pose tunnel = new Pose(117.39341421143848, 45.483535528596185);
                Pose toScore = new Pose(91.4367417677643, 15.246100519930673);

                Path toTunnel = new Path(new BezierLine(follower.getPose(), tunnel));
                Path toScorePath = new Path(new BezierLine(follower.getPose(), toScore));

                toTunnel.setHeadingInterpolation(HeadingInterpolator.facingPoint(scorePose));

                if (!follower.isBusy()) {
                    driveToClosestArtifact();
                    break;
                }

                // Step 3: score
                if (!visionPathActive) {
                    follower.followPath(toScorePath, true);
                    shoot();
                }
                break;

                 */

        }
    }

    public void shoot() {

        switch (shootBall) {
            case 0:
                follower.pausePathFollowing();
                launcher.setVelocity(targetVelocity);
                if (Math.abs(launcher.getVelocity() - realVelocity) < 70) {
                    trigger.setPosition(0);
                    intake.set(1);
                    actionTimer.resetTimer();
                    setShootBall(1);
                }
                break;
            case 1:
                follower.resumePathFollowing();
                if (actionTimer.getElapsedTimeSeconds() > 5) {
                    trigger.setPosition(0.2);
                    intake.set(0);
                    agitator.setPosition(1);
                    actionTimer.resetTimer();
                    setShootBall(2);
                }
                break;
            case 2:
                agitator.setPosition(0);
                launcher.set(0);
                setShootBall(3);
                break;
        }

    }

    public void setShootBall(int sBall) {
        shootBall = sBall;
        shootTimer.resetTimer();
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        setShootBall(0);
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        targetDistance = (Math.sqrt(Math.pow((follower.getPose().getX()-scorePose.getX()), 2)+(Math.pow(follower.getPose().getY() - scorePose.getY(), 2))));
        pitch.set(lut.get(targetDistance));

        if (follower.isBusy()) {
            telemetry.addLine("follower is busy");
        } else {
            telemetry.addLine("follower is finished");
        }

        telemetry.addData("actual launcher velocity", launcher.getVelocity());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path state", pathState);
        telemetry.addData("shoot ball", shootBall);

        telemetry.setAutoClear(true);

        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        launcher = new MotorEx(hardwareMap, "launcher");
        intake = new Motor(hardwareMap, "intake");
        trigger = hardwareMap.get(Servo.class, "trigger");
        light = hardwareMap.get(Servo.class, "light");
        pitch = new ServoEx(hardwareMap, "pitch", 0, 1800);
        agitator = hardwareMap.get(Servo.class, "agitator");

        shootBall = 0;

        //reverses directions for motors where it is necessary
        launcher.setInverted(true);
        intake.setInverted(true);

        launcher.setRunMode(MotorEx.RunMode.VelocityControl);

        //turns on brake mode
        //brake mode gives motors power in the opposite direction in order to make them stop faster
        //same technique is used in regenerative braking for electric vehicles
        launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        //ensures that all servos start at the correct position
        trigger.setPosition(.2);
        //sets pid values for the launcher
        launcher.setVeloCoefficients(0.6, 0, 0);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        shootTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        lut = new InterpLUT();

        lut.add(-50, 0);
        lut.add(0, 0);
        lut.add(40, 0);
        lut.add(57, 220);
        lut.add(69, 220);
        lut.add(112, 250);
        lut.add(135.0, 65.0);
        lut.add(137, 775);
        lut.add(175, 120);
        lut.add(204, 70.0);

        lut.createLUT();

        pitch.set(0);
        trigger.setPosition(.2);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setShootBall(0);

        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
    }
}