package org.firstinspires.ftc.teamcode.subsystems.turret;

import android.graphics.Bitmap;
import android.graphics.Camera;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntSupplier;

public class TurretIOReal implements TurretIO, TurretConstants {

    //the servos that control the angle of our shot
    Servo rapidRedirector, rapidRedirector2;
    //the actual shooter wheel (one motor on both sides attached to the same shaft)
    DcMotorEx artifactAccelerator, artifactAccelerator2;
    //the servos that turn our turret
    CRServo turretTurner1, turretTurner2;

    Intake intake;
    //the magnet sensor that acts as a limit switch for our turret
    DigitalChannel magnet;

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }
    }


    private AprilTagProcessor tagProcessor;
    private CameraStreamProcessor dashboardProcessor;

    public TurretIOReal(HardwareMap hwMap, Intake intake){

        this.intake = intake;
        artifactAccelerator = hwMap.get(DcMotorEx.class, "AA");
        artifactAccelerator2 = hwMap.get(DcMotorEx.class, "AA2");
        rapidRedirector = hwMap.get(Servo.class, "RR1");
        rapidRedirector2 = hwMap.get(Servo.class, "RR2");
        turretTurner1 = hwMap.get(CRServo.class, "TT1");
        turretTurner2 = hwMap.get(CRServo.class, "TT2");
        magnet = hwMap.get(DigitalChannel.class, "magnet");

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        dashboardProcessor = new CameraStreamProcessor();
        VisionPortal.Builder vBuilder = new VisionPortal.Builder();

        vBuilder.setCamera(hwMap.get(WebcamName.class, "W"));
        vBuilder.addProcessor(tagProcessor);
        vBuilder.addProcessor(dashboardProcessor);
        vBuilder.setCameraResolution(new Size(frameWidth, 720));
        vBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        magnet.setMode(DigitalChannel.Mode.INPUT);

        VisionPortal vision = vBuilder.build();

        FtcDashboard.getInstance().startCameraStream(dashboardProcessor, 0);
        vision.resumeStreaming();
    }

    @Override
    public void updateInputs (TurretIO.TurretIOInputs inputs){


            List<AprilTagDetection> result = tagProcessor.getDetections();
            if (!result.isEmpty()) {
                for (AprilTagDetection tag : result) {
                    if (tag.id == TAGID) {
                        inputs.tagDistance = tag.ftcPose.range;
                        inputs.tagX = tag.center.x;

                    }
                }
            }else{
                inputs.tagX = (double)frameWidth / 2;
            }

            // average velocity of the two motors attached to the shooting wheel
            inputs.artifactAcceleratorVelocity = ( artifactAccelerator2.getVelocity() - artifactAccelerator.getVelocity() ) / 2;

            inputs.aimError = ((inputs.tagX - ((double) frameWidth / 2)) / ((double) frameWidth / 2));

            if (!magnet.getState()){
                intake.resetTurretEncoder();
            }
            inputs.turretPos = intake.getTurretPos();

            inputs.redirectorPos = rapidRedirector.getPosition();

            //this returns FALSE when the turret is AT 0
            inputs.magnetState = magnet.getState();



    }

    //sets power to the turret servos
    @Override
    public void setPowerTurretTurner(double power){
        turretTurner1.setPower(power);
        turretTurner2.setPower(power);
    }

    //sets power to the shooting angle servos
    @Override
    public void setPosRapidRedirector(double pos){
        rapidRedirector.setPosition(pos);
        rapidRedirector2.setPosition(pos);
    }

    //sets the velocity of the shooting wheel
    @Override
    public void setVelocityArtifactAccelerator(double velocity){
        artifactAccelerator.setVelocity(-velocity);
        artifactAccelerator2.setVelocity(velocity);
    }

//    //true is false (when the turret is at the magnet, it returns false)
//    public boolean magnetValue(){
//        return magnet.getState();
//    }












}
