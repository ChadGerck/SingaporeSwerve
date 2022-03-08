package frc.robot;

import frc.robot.subsystems.Drivetrain;

public class SwerveMath {
    static double wheelXcos, wheelYsin, NWwheelX, NWwheelY, NEwheelX, NEwheelY, SWwheelX, SWwheelY, SEwheelX, SEwheelY, max, 
    NWwheelMag, NEwheelMag, SWwheelMag, SEwheelMag, NWwheelRot, NEwheelRot, SWwheelRot, SEwheelRot;
    static double rotAng = .70710678; 
    public static void ComputeSwerve(double finalAngle,double directMag, double rotMag, Boolean fixRotation) {
        wheelXcos = Math.cos(finalAngle / 57.2957795) * directMag; 
        wheelYsin = Math.sin(finalAngle / 57.2957795) * directMag;

        NWwheelX = wheelXcos+(rotAng* rotMag);
        NWwheelY = wheelYsin+(rotAng* rotMag);
        if(!fixRotation){NWwheelRot = Math.atan2(NWwheelY, NWwheelX) * 57.2957795;}
        NWwheelMag = Math.hypot(NWwheelX, NWwheelY);

        NEwheelX = wheelXcos+(rotAng*-rotMag);
        NEwheelY = wheelYsin+(rotAng*rotMag);
        if(!fixRotation){NEwheelRot = Math.atan2(NEwheelY, NEwheelX) * 57.2957795;}
        NEwheelMag = Math.hypot(NEwheelX, NEwheelY);

        SWwheelX = wheelXcos+(rotAng*rotMag);
        SWwheelY = wheelYsin+(rotAng*-rotMag);
        if(!fixRotation){SWwheelRot = Math.atan2(SWwheelY, SWwheelX) * 57.2957795;}
        SWwheelMag = Math.hypot(SWwheelX, SWwheelY);

        SEwheelX = wheelXcos+(rotAng* -rotMag);
        SEwheelY = wheelYsin+(rotAng* -rotMag);
        if(!fixRotation){SEwheelRot = Math.atan2(SEwheelY, SEwheelX) * 57.2957795;}
        SEwheelMag = Math.hypot(SEwheelX, SEwheelY);

        max=NWwheelMag;
        if(NEwheelMag>max){max=NEwheelMag;} 
        else if(SWwheelMag>max){max=SWwheelMag;}else if(SEwheelMag>max){max=SEwheelMag;}
        if(max>1){NWwheelMag/=max;NEwheelMag/=max;SWwheelMag/=max;SEwheelMag/=max;}

        Drivetrain.setModule("FL",NWwheelRot,NWwheelMag); Drivetrain.setModule("FR",NEwheelRot,NEwheelMag);
        Drivetrain.setModule("BL",SWwheelRot,SWwheelMag); Drivetrain.setModule("BR",SEwheelRot,SEwheelMag);
    }
}