package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionIOReal implements VisionIO, VisionConstants{

    private DcMotorEx turretEncoderVision;

    private Position cameraPosition = new Position(DistanceUnit.METER,
            centralCamDist, 0, camHeight, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS,
            0, camAngle, 0, 0);

    private AprilTagProcessor aprilTag;


    public VisionIOReal(HardwareMap hwMap){
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(800, 600));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        turretEncoderVision = hwMap.get(DcMotorEx.class, "intake");
        aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).setLensIntrinsics(fx,fy,cx,cy).build();
        builder.setCamera(hwMap.get(WebcamName.class, "W"));
        builder.addProcessor(aprilTag);
        VisionPortal visionPortal = builder.build();


        visionPortal.resumeStreaming();

    }

    @Override
    public void updateInputs(VisionIO.VisionIOInputs inputs) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (!detection.metadata.name.contains("Obelisk")) {
                    //X and Y converted to meters from inches
                    inputs.botX = ((detection.robotPose.getPosition().x) / 39.37) - (Math.cos(inputs.turretAngle) * centralCamDist);
                    inputs.botY = ((detection.robotPose.getPosition().y) / 39.37) - (Math.sin(inputs.turretAngle) * centralCamDist);
                    inputs.turretAngle = (turretEncoderVision.getCurrentPosition() * 0.00009) + Math.PI / 2;

                }
            }
        }

        inputs.distanceFromGoal = Math.sqrt(Math.pow(inputs.botX - 1.55, 2) + Math.pow(inputs.botY - (-1.55), 2));
    }

}
