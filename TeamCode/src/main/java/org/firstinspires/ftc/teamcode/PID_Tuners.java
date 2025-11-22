package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "test", group = "Testing")
@Configurable
public class PID_Tuners extends OpMode {
    public static int kp = 1;
    public static int ki = 1;
    public static int kd = 0;
    public static int kf = 0;
    private DcMotorEx motor = null;
    private Servo servo = null;
    private CRServo crservo = null;
    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        crservo = hardwareMap.get(CRServo.class, "crservo");

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kp, ki, kd, kf));
        motor.setPower(gamepad1.right_trigger);
        motor.setPower(-(gamepad1.left_trigger));

        crservo.setPower(gamepad1.left_stick_x);

        if (gamepad1.dpadUpWasPressed()) {
            servo.setPosition(0);
        } else if (gamepad1.dpadDownWasPressed()){
            servo.setPosition(1);
        }


    }
}