package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.I2CDevices.AdafruitNeoPixel;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.IntStream;

public class LightManager {

    static AdafruitNeoPixel driver; // LED Strip Driver Board
    static Servo gbLightTop, gbLightMiddle, gbLightBottom; // Big GoBilda Lights
    static ElapsedTime updateTime = new ElapsedTime();

    public static void inititalize(HardwareMap hardwareMap) {

        driver = hardwareMap.get(AdafruitNeoPixel.class, "NeoPixel");
        driver.initialize(LEDStrip.getLength(), 3);
        gbLightTop = hardwareMap.get(Servo.class, "TopLight");
        gbLightMiddle = hardwareMap.get(Servo.class, "MiddleLight");
        gbLightBottom = hardwareMap.get(Servo.class, "BottomLight");
        updateTime.reset();

    }

    public static class GoBildaLights {

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
    }

    public static class LEDStrip {

        public static void setLED(int index, int color) {
            driver.setLed(index, color);
        }

        public static void setLED(int index, int[] colors) {
            driver.setLed(index, colors);
        }

        public static void clear() {
            driver.clearLeds();
            driver.show(true);
        }

        public static int getLength() {
            return 2*LightConstants.visibleLength + LightConstants.gapLength;
        }

        public static void update() {
            driver.show(updateTime.seconds() > LightConstants.fullUpdatePeriod);
            if (updateTime.seconds() > LightConstants.fullUpdatePeriod) updateTime.reset();
        }

        public static String getStripConnInfo() {
            return driver.getConnectionInfo();
        }

        public static void setRPMLights(double motorRPM, double setpoint) {
            double motorRatio = (motorRPM / setpoint) * LightConstants.visibleLength;

            int[] bar = new int[LightConstants.visibleLength];

            for (int i = 0; i < bar.length; i++) {
                if (motorRatio < i) {
                    bar[i] = AdafruitNeoPixel.rgbToColor(0, 0,0);
                    continue;
                }
                if (i < 9) {
                    bar[i] = AdafruitNeoPixel.rgbToColor(255, 10, 10);
                } else if (i < 13) {
                    bar[i] = AdafruitNeoPixel.rgbToColor(255, 10, 96);
                } else {
                    bar[i] = AdafruitNeoPixel.rgbToColor(10, 10, 255);
                }
            }

            int[] barReversed = IntStream.range(0, bar.length).map(i -> bar[bar.length - 1 - i]).toArray();

            setLED(0, barReversed);
            setLED(LightConstants.visibleLength + LightConstants.gapLength, bar);

            update();

        }
    }

    public interface LightConstants {

        int gapLength = 8;
        int visibleLength = 18;

        double fullUpdatePeriod = 4;

        double Black = 0.0;
        double Red = 0.28;
        double Orange = 0.34;
        double Yellow = 0.39;
        double Green = 0.45;
        double Blue = 0.62;
        double Indigo = 0.66;
        double Purple = 0.72;
        double White = 1.0;
    }


}
