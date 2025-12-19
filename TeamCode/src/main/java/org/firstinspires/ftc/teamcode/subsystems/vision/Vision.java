package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Vision extends SubsystemBase {

    private VisionIO io;

    private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
    public Vision(VisionIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }


    public double getBotX(){
        return inputs.botX;
    }

    public double getBotY() {
        return inputs.botY;
    }

    public double getDistanceFromGoal(){
        return inputs.distanceFromGoal;
    }




















}
