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
    public void rapidRedirect(double degrees) {
//       io.setPosRapidRedirector(-0.033 * (Math.tan(-6.96 * (Math.toDegrees(radians) - 117.83))) + 0.439);
       io.setPosRapidRedirector((degrees - 41.1699) / -46.45631);
    }
//    ((degrees - 41.1699) / -46.45631)
//    ((0.02176 * degrees)- 1.064)



    //Math.PI / 2 - radians) * -0.0194342 + 1.16342)

    public void reticleRapidRedirectorRegression() {
        if(inputs.redirectorPos > 0) {
            rapidRedirect(0.275 * Math.pow((nabNormal() + 0.734), -0.98) + 7.46 - (2 * Math.PI));
        }else{
            rapidRedirect(41);
        }
}

    public double nabNormal() {
        try {
            return (((inputs.tagDistance / 39.37) * 1.709) - 0.19);
        }catch(NullPointerException e){
            return 0.0;
        }

    }

    public boolean magnetState(){
        return inputs.magnetState;
    }

    public double procurePlatePosition(){
        return inputs.turretPos;
    }

    public double attainAccelerationAntiderivative() {
        return inputs.artifactAcceleratorVelocity;
    }

    public void accelerateArtifact(double accelAntiderivative){
        io.setVelocityArtifactAccelerator(accelAntiderivative);
    }




    public void autoAccelerateArtifact(){
        accelerateArtifact(1000);
    }
    //1900 far

    public double shooterVelocity(){
       return inputs.artifactAcceleratorVelocity;
    }

    public void turretTurn(double power){
        io.setPowerTurretTurner(power);
    }

    public void redirect(double pos){
        io.setPosRapidRedirector(pos);
    }



}
