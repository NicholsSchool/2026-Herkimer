package org.firstinspires.ftc.teamcode.testTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.I2CDevices.AdafruitNeoPixel;
import org.firstinspires.ftc.teamcode.subsystems.LightManager;

@TeleOp(name = "Light Strip Test")
public class LightStripVisTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        LightManager.inititalize(hardwareMap);

        ElapsedTime time = new ElapsedTime();

        time.reset();

        LightManager.GoBildaLights.setLights(new double[] {LightManager.LightConstants.Red, LightManager.LightConstants.Green, LightManager.LightConstants.Blue});

        while (opModeInInit()) {
            telemetry.addData("LED CONN INFO", LightManager.LEDStrip.getStripConnInfo());
            telemetry.update();
        }

        while (opModeIsActive()) {
            LightManager.LEDStrip.setRPMLights(Math.abs(Math.sin(time.seconds())), 1);
        }

    }


}
