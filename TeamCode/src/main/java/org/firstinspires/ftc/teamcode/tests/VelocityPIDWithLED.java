package org.firstinspires.ftc.teamcode.tests;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math_utils.PIDController;

@TeleOp(name = "VelocityPID")
@Config
public class VelocityPIDWithLED extends OpMode{


    DcMotorEx turretFlywheel;
    PIDController pidController;
    public static double setPoint = 2000;
    public static double kP, kI, kD;
    FtcDashboard dashboard;

    @Override
    public void init() {

        kP = 0.1;
        kI = 0.0;
        kD = 0.0;

        turretFlywheel = hardwareMap.get(DcMotorEx.class, "turretFlywheel");
        pidController = new PIDController(kP,kI,kD);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

    }

    @Override
    public void loop() {

        turretFlywheel.setPower(pidController.calculate(getMotorRPM(), setPoint));

        telemetry.addData("FlywheelVelocity", getMotorRPM());
        telemetry.addData("Setpoint", setPoint);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);

    }

    public double getMotorRPM (){
        return turretFlywheel.getVelocity() * 28;
    }
}
