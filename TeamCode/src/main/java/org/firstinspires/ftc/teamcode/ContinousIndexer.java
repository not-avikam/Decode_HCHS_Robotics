package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;


@Configurable
@TeleOp(name = "cont spindex test", group = "Testing")
public class ContinousIndexer extends OpMode {
    private CRServoEx indexer = null;
    private NormalizedColorSensor colorSensorIntake = null;
    private NormalizedColorSensor colorSensorShoot = null;
    private enum IndexIntake {
        INTAKE_1,
        INTAKE_2,
        INTAKE_3
    }

    private enum IndexShoot {
        SHOOT_1,
        SHOOT_2,
        SHOOT_3
    }
    double hueShoot, hueIntake;
    private IndexIntake indexIntake = IndexIntake.INTAKE_1;
    private IndexShoot indexShoot = IndexShoot.SHOOT_1;
    private TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void init() {

        indexer = new CRServoEx(hardwareMap, "indexer");
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "sensor_intake");
        colorSensorShoot = hardwareMap.get(NormalizedColorSensor.class, "sensor_shoot");

    }
    @Override
    public void loop() {

        NormalizedRGBA colorsIntake = colorSensorIntake.getNormalizedColors();
        NormalizedRGBA colorsShoot = colorSensorShoot.getNormalizedColors();
        hueIntake = JavaUtil.colorToHue(colorsIntake.toColor());
        hueShoot = JavaUtil.colorToHue(colorsShoot.toColor());

        if (gamepad1.dpadRightWasPressed()) {
            switch (indexIntake) {
                //TODO: fix hue values
                //TODO: EXTREMELY IMPORTANT
                case INTAKE_1:
                    indexer.set(0);
                    indexIntake = IndexIntake.INTAKE_2;
                    //yellow
                    if (hueShoot > 60 && hueShoot < 150){
                        indexer.set(0);
                    } else {
                        indexer.set(1);
                    }
                    break;
                case INTAKE_2:
                    //orange
                    indexIntake = IndexIntake.INTAKE_3;
                    if (hueShoot > 30 && hueShoot < 90){
                        indexer.set(0);
                    } else {
                        indexer.set(1);
                    }
                    break;
                case INTAKE_3:
                    //blue
                    if (hueShoot > 150 && hueShoot < 350){
                        indexer.set(0);
                    } else {
                        indexer.set(1);
                    }
                    indexIntake = IndexIntake.INTAKE_1;
                    break;
            }
        } else if (gamepad1.dpadLeftWasPressed()) {
            switch (indexIntake) {
                case INTAKE_1:
                    indexIntake = IndexIntake.INTAKE_2;
                    //blue
                    if (hueShoot > 150 && hueShoot < 350){
                        indexer.set(0);
                    } else {
                        indexer.set(1);
                    }
                    break;
                case INTAKE_2:
                    indexIntake = IndexIntake.INTAKE_3;
                    //orange
                    if (hueShoot > 30 && hueShoot < 90){
                        indexer.set(0);
                    } else {
                        indexer.set(1);
                    }
                    break;
                case INTAKE_3:
                    //yellow
                    if (hueShoot > 60 && hueShoot < 150){
                        indexer.set(1);
                    } else {
                        indexer.set(0);
                    }
                    indexIntake = IndexIntake.INTAKE_1;
                    break;
            }
        }

        if (gamepad1.dpadUpWasPressed()) {
            switch (indexShoot) {
                case SHOOT_1:
                    //yellow
                    indexShoot = IndexShoot.SHOOT_2;
                    if (hueIntake > 60 && hueIntake < 150){
                        indexer.set(1);
                    } else {
                        indexer.set(0);
                    }
                    break;
                case SHOOT_2:
                    indexShoot = IndexShoot.SHOOT_3;
                    //orange
                    if (hueIntake > 30 && hueIntake < 90){
                        indexer.set(1);
                    } else {
                        indexer.set(0);
                    }
                    break;
                case SHOOT_3:
                    //blue
                    if (hueIntake > 150 && hueIntake < 350){
                        indexer.set(0);
                    } else {
                        indexer.set(1);
                    }
                    indexShoot = IndexShoot.SHOOT_1;
                    break;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            switch (indexShoot) {
                case SHOOT_1:
                    indexShoot = IndexShoot.SHOOT_2;
                    //blue
                    if (hueIntake > 150 && hueIntake < 350){
                        indexer.set(0);
                    } else {
                        indexer.set(1);
                    }
                    break;
                case SHOOT_2:
                    indexShoot = IndexShoot.SHOOT_3;
                    //orange
                    if (hueIntake > 30 && hueIntake < 90){
                        indexer.set(1);
                    } else {
                        indexer.set(0);
                    }
                    break;
                case SHOOT_3:
                    indexShoot = IndexShoot.SHOOT_1;
                    //yellow
                    if (hueIntake > 60 && hueIntake < 150){
                        indexer.set(1);
                    } else {
                        indexer.set(0);
                    }
                    break;
            }
        }

        panelsTelemetry.addData("intake hue", hueIntake);
        panelsTelemetry.addData("shoot hue", hueShoot);
        panelsTelemetry.update(telemetry);

    }
}