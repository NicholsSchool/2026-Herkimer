package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface DrivetrainConstants {

    public final double red = 0.277;
    public final double orange = 0.333;
    public final double yellow = 0.388;
    public final double green = 0.444;
    public final double blue = 0.611;
    public final double indigo = 0.666;
    public final double ourple = 0.722;

         double fL_P = 0.1;
         double fL_I = 0.0;
         double fL_D = 0.0;

         double fR_P = 0.1;
         double fR_I = 0.0;
         double fR_D = 0.0;

         double bL_P = 0.1;
         double bL_I = 0.0;
         double bL_D = 0.0;

         double bR_P = 0.1;
         double bR_I = 0.0;
         double bR_D = 0.0;

         double motorRPM = 2000; //TODO: Change values to be more correcter
         double ticksPerRot = 200;
}
