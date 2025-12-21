package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LightManager;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

@Disabled
@TeleOp (name = "servoTest")
public class turretServoTestTele extends OpMode {

    Turret turret;
    @Override
    public void init() {
        LightManager.inititalize(hardwareMap);
        turret = new Turret(new TurretIOReal(hardwareMap));

    }

    @Override
    public void loop() {
        turret.periodic();
        turret.turretSetPower(gamepad1.left_stick_x);
    }
}
