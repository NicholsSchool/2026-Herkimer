package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.I2CDevices.AdafruitNeoPixel;

public class LightManager {

    static AdafruitNeoPixel driver; // LED Strip Driver Board
    static Servo gbLightTop, gbLightMiddle, gbLightBottom; // Big GoBilda Lights
    static ElapsedTime time;

    public static void inititalize(HardwareMap hardwareMap) {

        driver = hardwareMap.get(AdafruitNeoPixel.class, "NeoPixel");
        driver.initialize(30, 3);
        gbLightTop = hardwareMap.get(Servo.class, "TopLight");
        gbLightMiddle = hardwareMap.get(Servo.class, "MiddleLight");
        gbLightBottom = hardwareMap.get(Servo.class, "BottomLight");
        time.reset();

    }

    public static void setRPMLights(double motorRPM) {
        motorRPM = Range.clip(motorRPM, 0.0, 6000.0);


    }

    public interface LightConstants {

        int startingOffset = 0;
        int visibleLength = 18;

    }


}
