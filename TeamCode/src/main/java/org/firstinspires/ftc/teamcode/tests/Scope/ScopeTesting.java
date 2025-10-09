package org.firstinspires.ftc.teamcode.tests.Scope;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SwerveTele")
public class ScopeTesting extends OpMode {

    Arm arm;

    @Override
    public void init() {

        arm = new Arm(new ArmIOReal());

    }

    @Override
    public void loop() {
        configureButtonBindings();
        updateSubsystems();

    }

    public void configureButtonBindings(){
        //operator buttons
        arm.manualPos(gamepad1.left_stick_y);
        if(gamepad1.a) {
            arm.runToPos(1);
        }
    }

    public void updateSubsystems(){
        arm.periodic();
    }
}
