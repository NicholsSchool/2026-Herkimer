package org.firstinspires.ftc.teamcode.testTeleops;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.I2CDevices.AdafruitNeoPixel;
@Disabled
@TeleOp
public class RandomColorsTest extends OpMode {

    private static final int NUM_PIXELS = 50;

    ColorSensor cs;
    AdafruitNeoPixel neoPixel;

    @Override
    public void init() {
        neoPixel = hardwareMap.get(AdafruitNeoPixel.class, "NeoPixel");
    }

    @Override
    public void start() {
        neoPixel.initialize(NUM_PIXELS, 3);
        neoPixel.clearLeds();
        neoPixel.show(true);
    }

    @Override
    public void loop() {

        for (int i = 0; i < NUM_PIXELS; i++) {
            if ( i > NUM_PIXELS * gamepad1.right_trigger) {
                neoPixel.setLed(i, Color.BLACK);
            } else {
                neoPixel.setLed(i, AdafruitNeoPixel.rgbToColor((int) (((float) i / (float) NUM_PIXELS) * 255), 0, 255 - (int) (((float) i / (float) NUM_PIXELS) * 255)));
            }
        }

        neoPixel.show();

    }

    public void stop() {
        neoPixel.clearLeds();
        neoPixel.show(true);
    }
}