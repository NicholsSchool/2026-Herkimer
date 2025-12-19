package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

public interface DrivetrainConstants {

        double pwmRed = 0.277;
        double pwmOrange = 0.333;
        double pwmYellow = 0.388;
        double pwmGreen = 0.444;
        double pwmBlue = 0.611;
        double pwmIndigo = 0.666;
        double pwmPurple = 0.722;

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

         double DRIVE_SETPOINT_THRESHOLD = 0.03;
         double TURN_SETPOINT_THRESHOLD = 4.0;


}
