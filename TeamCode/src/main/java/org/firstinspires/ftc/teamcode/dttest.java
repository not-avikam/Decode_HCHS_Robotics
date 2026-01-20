package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp(name = "dt test", group = "lm2 2025")
public class dttest extends OpMode {


    private Motor frontLeftDrive = null;
    private Motor backLeftDrive = null;
    private Motor frontRightDrive = null;
    private Motor backRightDrive = null;
    MecanumDrive mecanum;
    IMU imu;

    @Override
    public void init() {

        frontLeftDrive = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        backLeftDrive = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312);
        frontRightDrive = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        backRightDrive = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312);
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftDrive.setRunMode(Motor.RunMode.RawPower);
        frontRightDrive.setRunMode(Motor.RunMode.RawPower);
        backLeftDrive.setRunMode(Motor.RunMode.RawPower);
        backRightDrive.setRunMode(Motor.RunMode.RawPower);

        frontRightDrive.setInverted(true);
        backRightDrive.setInverted(true);

        mecanum = new MecanumDrive(
                false,
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive
        );


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // set the run mode

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        mecanum.driveRobotCentric(
                -gamepad1.left_stick_y,  // forward
                gamepad1.left_stick_x,  // strafe
                gamepad1.right_stick_x  // turn
        );


        /*

        // DIAGNOSTIC TEST: Press buttons to see which wheel spins
        if (gamepad1.a) frontLeftDrive.set(0.5);   // Should be Front Left
        if (gamepad1.b) frontRightDrive.set(0.5);  // Should be Front Right
        if (gamepad1.x) backLeftDrive.set(0.5);    // Should be Back Left
        if (gamepad1.y) backRightDrive.set(0.5);   // Should be Back Right
*/




        telemetry.update();

    }
}