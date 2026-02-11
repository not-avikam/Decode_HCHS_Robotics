package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
@TeleOp(name = "lcq tuners", group = "lcq")
public class lcq_tuners extends OpMode {

    private ServoEx indexer;
    private MotorEx intake;
    //
    //private ServoEx pitch;
    private ServoEx trigger;

    double position1 = 0;
    double position2 = 240;
    double position3 = 300;
    double targetPosition = 0;
    private MotorEx launcher;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;
    public static double kV = 0;
    @Override
    public void init() {

       indexer = new ServoEx(hardwareMap, "pitch", 0, 1800);
       launcher = new MotorEx(hardwareMap, "launcher");
       trigger = new ServoEx(hardwareMap, "trigger");
       intake = new MotorEx(hardwareMap, "intake");

       intake.setInverted(true);

       launcher.setInverted(true);
       launcher.setRunMode(MotorEx.RunMode.VelocityControl);
       launcher.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        SimpleMotorFeedforward feedforward =
                new SimpleMotorFeedforward(kS, kV);

        launcher.setVeloCoefficients(kP, kI, kD);

        if (gamepad1.left_trigger != 0) {
            launcher.setVelocity(feedforward.calculate(1500));
        }

        if (gamepad1.right_trigger != 0) {
            trigger.set(1);
        } else {
            trigger.set(0);
        }

        if (gamepad1.right_bumper) {
            intake.set(1);
        } else {
            intake.set(0);
        }

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

        if (gamepad1.dpadUpWasPressed()) {
            indexer.set(targetPosition);
        }

        telemetry.addData("launcher pace", launcher.getVelocity());
        telemetry.addData("targetPosition", targetPosition);

        telemetry.update();


        //return 0;
    }
}