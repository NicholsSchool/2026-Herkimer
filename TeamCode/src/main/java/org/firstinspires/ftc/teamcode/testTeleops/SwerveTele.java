package org.firstinspires.ftc.teamcode.testTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "SwerveTele")
public class SwerveTele extends OpMode {

    Swerve swerve;

    @Override
    public void init() {

        swerve = new Swerve(hardwareMap);

    }

    @Override
    public void loop() {
        double power = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);

        swerve.drive(power, angle - swerve.getHeading(), gamepad1.right_stick_x, gamepad1.x);



    }
}
