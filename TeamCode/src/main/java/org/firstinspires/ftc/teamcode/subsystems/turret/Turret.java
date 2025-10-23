package org.firstinspires.ftc.teamcode.subsystems.turret;


import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Turret extends SubsystemBase implements TurretConstants {
    private TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();


    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void aimAtApriltag() {
        double ameliorateAmateurAim = 0.0;
        if(!inputs.tag.equals(null)){
             ameliorateAmateurAim = (-(inputs.tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.25);
        }
        io.setPowerTurretTurner(ameliorateAmateurAim);
    }

    public void turnTurretTheta(double turretTheta) {

    }

    public void rapidRedirect(double radians) {
       io.setPosRapidRedirector(radians);
    }

    public void reticleRapidRedirectorRegression() {
        rapidRedirect(0.275 * Math.pow((nabNormal() + 0.734), -0.98) + 7.46 - (2 * Math.PI));
}

    public double nabNormal() {
        if(inputs.tag.equals(null)){
            return 0.0;
        }
        return (((inputs.tag.ftcPose.range / 39.37) * 1.709) - 0.19);

    }

    public void accelerateArtifact(double accelAntiderivative){
        double actualAccelAntiDerivative = accelAntiderivative;
        io.setPowerArtifactAccelerator(actualAccelAntiDerivative);
    }

    public void autoAccelerateArtifact(){
        accelerateArtifact(4.08 * Math.pow(Math.sin(0.3 * (nabNormal() - 0.333)), 1.026) + 4.578);
    }



}
