package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface DrivetrainConstants {

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
