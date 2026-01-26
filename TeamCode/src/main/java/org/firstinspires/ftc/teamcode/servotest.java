package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

@Configurable
@TeleOp(name = "servo test", group = "Testing")
public class servotest extends OpMode {

    private ServoEx indexer;
    //
    //private ServoEx pitch;

    double position1 = 0;
    double position2 = 240;
    double position3 = 300;
    double targetPosition = 0;

    double[] INTAKE_POS = {0, 130, 270};
    double[] SHOOT_POS = {position1, position2, position3};

    int i = 0;
    int s = 0;

    @Override
    public void init() {

       indexer = new ServoEx(hardwareMap, "pitch", 0, 1800);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {


        if (gamepad1.yWasPressed()) {
            targetPosition += 5;
            position2 += 5;
            position3 += 5;
        } else if (gamepad1.aWasPressed()) {
            targetPosition -= 5;
            position2 -= 5;
            position3 -= 5;
        } else if (gamepad1.bWasPressed()) {
            targetPosition += 100;
            position2 += 100;
            position3 += 100;
        } else if (gamepad1.xWasPressed()) {
            targetPosition -= 100;
            position2 -= 100;
            position3 -= 100;
        }






/*
        // Indexer control
        if (gamepad1.dpadRightWasPressed()) {
            if (i < INTAKE_POS.length - 1) i++;
            indexer.set(INTAKE_POS[i]);
        } else if (gamepad1.dpadLeftWasPressed()) {
            if (i > 0) i--;
            indexer.set(INTAKE_POS[i]);
        } else if (gamepad1.dpadUpWasPressed()) {
            if (s < SHOOT_POS.length - 1) s++;
            indexer.set(SHOOT_POS[s]);
        } else if (gamepad1.dpadDownWasPressed()) {
            if (s > 0) s--;
            indexer.set(SHOOT_POS[s]);
        }


 */
        if (gamepad1.dpadUpWasPressed()) {
            indexer.set(targetPosition);
        }

        telemetry.addData("intake pos", INTAKE_POS[i]);
        telemetry.addData("shoot pos", SHOOT_POS[s]);
        telemetry.addData("targetPosition", targetPosition);

        telemetry.update();


        //return 0;
    }

}