package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
@Configurable
@TeleOp(name = "intake test", group = "Testing")
public class intaketest extends OpMode {
    private DcMotorEx motor = null;
    private Servo servo = null;
    private CRServo crservo = null;
    public static double power = 0;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void init() {

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        crservo = hardwareMap.get(CRServo.class, "crservo");

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        motor.setPower(power);
        crservo.setPower(gamepad1.left_stick_x);

        if (gamepad1.dpadUpWasPressed()) {
            servo.setPosition(0);
        } else if (gamepad1.dpadDownWasPressed()){
            servo.setPosition(1);
        }

        panelsTelemetry.addData("motor velocity", motor.getVelocity());

        panelsTelemetry.update(telemetry);

        //return 0;
    }
}