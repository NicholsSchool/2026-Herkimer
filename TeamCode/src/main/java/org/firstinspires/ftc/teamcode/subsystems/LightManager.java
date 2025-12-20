package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.I2CDevices.AdafruitNeoPixel;

public class LightManager {

    static AdafruitNeoPixel driver; // LED Strip Driver Board
    static Servo gbLightTop, gbLightMiddle, gbLightBottom; // Big GoBilda Lights
    static ElapsedTime time = new ElapsedTime();

    public static void inititalize(HardwareMap hardwareMap) {

        driver = hardwareMap.get(AdafruitNeoPixel.class, "NeoPixel");
        driver.initialize(30, 3);
        gbLightTop = hardwareMap.get(Servo.class, "TopLight");
        gbLightMiddle = hardwareMap.get(Servo.class, "MiddleLight");
        gbLightBottom = hardwareMap.get(Servo.class, "BottomLight");
        time.reset();

    }

    public static void setTopLight(double color) {
        gbLightTop.setPosition(color);
    }

    public static void setMiddleLight(double color) {
        gbLightMiddle.setPosition(color);
    }

    public static void setBottomLight(double color) {
        gbLightBottom.setPosition(color);
    }

    public static void setLights(double[] color) {
        setTopLight(color[0]);
        setMiddleLight(color[1]);
        setBottomLight(color[2]);
    }

    public static void setRPMLights(double motorRPM) {
        motorRPM = Range.clip(motorRPM, 0.0, 6000.0);
    }

    public interface LightConstants {

        int startingOffset = 0;
        int visibleLength = 18;

        double Red = 0.277;
        double Orange = 0.333;
        double Yellow = 0.388;
        double Green = 0.444;
        double Blue = 0.611;
        double Indigo = 0.666;
        double Purple = 0.722;

    }


}
