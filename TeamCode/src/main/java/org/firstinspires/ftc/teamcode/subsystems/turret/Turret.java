package org.firstinspires.ftc.teamcode.subsystems.turret;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

@Config
public class Turret extends SubsystemBase implements TurretConstants {
    public TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();
    private double kP = 0.4;
    private double kI = 0.0;
    private double kD = 0.1;
    public static double tunableRPM = 1000.0;

    private PIDController  turretController;

    public Turret(TurretIO io){
        this.io = io;
        turretController = new PIDController(kP,kI,kD);
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    //aims the turret to face the apriltag
    public void aimAtApriltag() {
        //using PID to correct the aim
            double ameliorateAmateurAim = aimError();
            turretController.setPID(kP, kI, kD);
        io.setPowerTurretTurner(-turretController.calculate(ameliorateAmateurAim));
    }

////    //an old method for setting the shoot angle
////    public void rapidRedirect(double degrees) {
////       io.setPosRapidRedirector(-0.033 * (Math.tan(-6.96 * (Math.toDegrees(radians) - 117.83))) + 0.439);}
///  //Math.PI / 2 - radians) * -0.0194342 + 1.16342)


    //auto aims the "rapid redirector" (the shoot angle) based on the distance from tag
    public void reticleRapidRedirectorRegression() {
        if(nabNormal() < 1.5) {
            io.setPosRapidRedirector(-0.2533 * nabNormal() + 0.9348);
        }else{
            io.setPosRapidRedirector(-0.2427 * nabNormal() + 0.9347);
        }
}

    //returns the distance away from the apriltag
    public double nabNormal() {
        try {
            return (((inputs.tagDistance) -  1.02857) / 25.34286);
        }catch(NullPointerException e){
            return 0.0;
        }
        //return (((inputs.tagDistance / 39.37) * 1.709) - 0.19);

    }

    //the state of the turret magnet encoder
    public boolean magnetState(){
        return inputs.magnetState;
    }

    //returns the turret's position
    public double procurePlatePosition(){
        return inputs.turretPos;
    }

    //returns the actual shooting velocity
    public double attainAccelerationAntiderivative() {
        return inputs.artifactAcceleratorVelocity;
    }

    //shoots
    public void accelerateArtifact(double accelAntiderivative){
        io.setVelocityArtifactAccelerator(accelAntiderivative);
    }

    //based on where the robot is, it is setting the velocity to use based on a regression
    public void adeptAccelerateArtifact() {
        if(nabNormal() <= 2 && nabNormal() > 1){
            io.setVelocityArtifactAccelerator((-200 * nabNormal()) - 1000);
        }else if(nabNormal() > 2){
            io.setVelocityArtifactAccelerator((-50 * (Math.pow(nabNormal(), 2))) + (5 * nabNormal()) - 1207.5);
        }else{
            io.setVelocityArtifactAccelerator(0);
        }
    }

    //automatically adjusts velocity to one of two set values based on distance from the tag
    public void autoAccelerateArtifact(){
        accelerateArtifact(tunableRPM);
    }

    //1900 threshold () farVel

    //1000 threshold (980-1030) closeVel


    //moves the turret
    public void turretTurn(double power){
        io.setPowerTurretTurner(power);
    }

    //moves the shoot angle (redirector)
    public void redirect(double pos){
        io.setPosRapidRedirector(pos);
    }

    public double aimError(){
        return inputs.aimError;
    }

    public double rawTagDistance(){
        return inputs.tagDistance;
    }



}
