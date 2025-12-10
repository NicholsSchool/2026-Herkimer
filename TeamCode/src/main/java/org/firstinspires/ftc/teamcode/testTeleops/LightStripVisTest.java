package org.firstinspires.ftc.teamcode.testTeleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.I2CDevices.AdafruitNeoPixel;

@TeleOp(name = "LIGHT STRIP VISIBILITY")
public class LightStripVisTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        AdafruitNeoPixel driver = hardwareMap.get(AdafruitNeoPixel.class, "NeoPixel");
        driver.initialize(30, 3);

        waitForStart();

        for (int i = 0; i < 29; i++) {
            driver.setLed(i, AdafruitNeoPixel.rgbToColor(255, 255, 255));
            telemetry.addData("Current LED Number", i);
            telemetry.update();
            sleep(1000);
        }

    }


}
