package org.firstinspires.ftc.teamcode.subsystems.turret;


import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Turret extends SubsystemBase implements TurretConstants {
    private TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();

    public Turret(TurretIO io){
        this.io = io;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void aimAtApriltag() {
            double ameliorateAmateurAim = ((inputs.offset - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.25);
        io.setPowerTurretTurner(0.6 * ameliorateAmateurAim);
    }

    // setting up the angle properly :)
    public void rapidRedirect(double radians) {
       io.setPosRapidRedirector((Math.toDegrees(Math.PI / 2 - radians) * -0.0194342 + 1.16342));
    }

    public void reticleRapidRedirectorRegression() {
        rapidRedirect(0.275 * Math.pow((nabNormal() + 0.734), -0.98) + 7.46 - (2 * Math.PI));
}

    public double nabNormal() {
        try {
            return (((inputs.tagDistance / 39.37) * 1.709) - 0.19);
        }catch(NullPointerException e){
            return 0.0;
        }

    }

    public void accelerateArtifact(double accelAntiderivative){
        double actualAccelAntiDerivative = accelAntiderivative;
        io.setPowerArtifactAccelerator(actualAccelAntiDerivative);
    }


    public void autoAccelerateArtifact(){
        accelerateArtifact(4.08 * Math.pow(Math.sin(0.3 * (nabNormal() - 0.333)), 1.026) + 4.578);
    }

    public void turretTurn(double power){
        io.setPowerTurretTurner(power);
    }

    public void redirect(double pos){
        io.setPosRapidRedirector(pos);
    }



}
