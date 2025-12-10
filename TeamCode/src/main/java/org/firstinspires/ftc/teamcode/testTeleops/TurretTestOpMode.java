package org.firstinspires.ftc.teamcode.testTeleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TURRET TEST")
public class TurretTestOpMode extends OpMode {

    private DcMotorEx fR, fL, bR, bL;
    TurretTest turret;



    public void drive(double power, double strafe, double turn) {

        fL.setPower((power ) - turn);
        fR.setPower((-power ) - turn);
        bR.setPower((-power ) - turn);
        bL.setPower((power ) - turn);
    }

    @Override
    public void init() {

        fL = hardwareMap.get(DcMotorEx.class, "fL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        bR = hardwareMap.get(DcMotorEx.class, "bR");

        turret = new TurretTest(hardwareMap);


    }

    @Override
    public void loop() {

        turret.apriltagAttributes();


        turret.aimAtApriltag();

//        turretMotor.setPower(-(tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.25);

        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Distance", (turret.nabNormal()));


    }
}
