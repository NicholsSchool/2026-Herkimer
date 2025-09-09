package org.firstinspires.ftc.teamcode.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TURRET TEST")
public class TurretTestOpMode extends OpMode {

    private DcMotor turretMotor;
    private DcMotorEx fR, fL, bR, bL;

    private final int TAGID = 20;
    private final int frameWidth = 1280;

    private AprilTagProcessor tagProcessor;

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

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal.Builder vBuilder = new VisionPortal.Builder();

        vBuilder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        vBuilder.addProcessor(tagProcessor);
        vBuilder.setCameraResolution(new Size(frameWidth, 720));
        vBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        VisionPortal vision = vBuilder.build();

        vision.resumeStreaming();

    }

    @Override
    public void loop() {

        List<AprilTagDetection> result = tagProcessor.getDetections();

        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (!result.isEmpty()) {
            for (AprilTagDetection tag : result) {
                if (tag.id == TAGID) {
                    telemetry.addData("TAG OUT", tag.center.x);
                    turretMotor.setPower(-(tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.25);
                }
            }
        } else {
            telemetry.addData("TAG OUT", "NONE");
            turretMotor.setPower(0);
        }

    }
}
