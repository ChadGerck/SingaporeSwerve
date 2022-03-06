package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;

public class OI{
    private static final double DEADZONE_LIMIT = 0.1;
	public static final String LeftY = null;
    public final XboxController Controller0 = new XboxController(0), Controller1 = new XboxController(1);
    public XboxController control; 
    public void Remote(int x){if(x==1){control=Controller0;}else if(x==2){control=Controller1;}}
    

    public double RightArc(int x){ Remote(x); return Math.toDegrees(Math.atan2(RightY(x), RightX(x))) + 90; }
    public double LeftArc(int x){ Remote(x); return Math.toDegrees(Math.atan2(LeftY(x), LeftX(x))) + 90; }
    public double getLeftJoystickAngle(int x){ Remote(x); return Math.toDegrees(Math.atan2(control.getRawAxis(0), -control.getRawAxis(1))); }
    public double LeftMag (int x){ Remote(x); return Math.hypot(control.getRawAxis(1), control.getRawAxis(0)); }
    public double RightMag(int x){ Remote(x); return Math.hypot(control.getRawAxis(4), control.getRawAxis(5)); }

    public double LeftX  (int x){ Remote(x); double raw = control.getRawAxis(0); return Math.abs(raw) < DEADZONE_LIMIT ? 0.0 : raw; }
    public double LeftY  (int x){ Remote(x); double raw = control.getRawAxis(1); return Math.abs(raw) < DEADZONE_LIMIT ? 0.0 : raw; }
    public double RightX (int x){ Remote(x); double raw = control.getRawAxis(4); return Math.abs(raw) < DEADZONE_LIMIT ? 0.0 : raw; }
    public double RightY (int x){ Remote(x); double raw = control.getRawAxis(5); return Math.abs(raw) < DEADZONE_LIMIT ? 0.0 : raw; }
    public double LeftTrigger (int x){ Remote(x); double raw = control.getRawAxis(2); return Math.abs(raw) < DEADZONE_LIMIT ? 0.0 : raw; }
    public double RightTrigger(int x){ Remote(x); double raw = control.getRawAxis(3); return Math.abs(raw) < DEADZONE_LIMIT ? 0.0 : raw; }
    
    public boolean LeftBumper     (int x){ Remote(x); return control.getLeftBumperPressed(); }
    public boolean LeftBumperDown (int x){ Remote(x); return control.getLeftBumper(); }
    public boolean RightBumper    (int x){ Remote(x); return control.getRightBumperPressed(); }
    public boolean RightBumperDown(int x){ Remote(x); return control.getRightBumper(); }
    public boolean AButton        (int x){ Remote(x); return control.getAButtonPressed(); }
    public boolean AButtonDown    (int x){ Remote(x); return control.getAButton(); }
    public boolean BButton        (int x){ Remote(x); return control.getBButtonPressed(); }
    public boolean BButtonDown    (int x){ Remote(x); return control.getBButton(); }
    public boolean XButton        (int x){ Remote(x); return control.getXButtonPressed(); }
    public boolean XButtonDown    (int x){ Remote(x); return control.getXButton(); }
    public boolean YButton        (int x){ Remote(x); return control.getYButtonPressed(); }
    public boolean YButtonDown    (int x){ Remote(x); return control.getYButton(); }
    public boolean StartButton    (int x){ Remote(x); return control.getStartButtonPressed(); }
    public boolean BackButton     (int x){ Remote(x); return control.getRawButtonPressed(7); }
    public boolean LSClick        (int x){ Remote(x); return control.getLeftStickButtonPressed(); }
    public boolean RSClick        (int x){ Remote(x); return control.getRightStickButtonPressed(); }
    public boolean LSClickDown    (int x){ Remote(x); return control.getLeftStickButton(); }
    public boolean RSClickDown    (int x){ Remote(x); return control.getRightStickButton(); }
    public double  Dpad           (int x){ Remote(x); return control.getPOV(); }

    public double LEDValue() { return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(0).doubleValue(); }
    public void LEDOn() { NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); } 
    public void LEDOff() { NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); } 
    public double LimelightTx() { return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); }
    
    public boolean DpadUp(int x){ Remote(x); double POV = control.getPOV(); 
        if((POV >= 0 && POV < 45) || (POV >= 315 && POV < 360)) { return true; } else { return false; }
    } public boolean DpadRight(int x){ Remote(x); double POV = control.getPOV(); 
        if(POV >= 45 && POV < 135) { return true; } else { return false; }
    } public boolean DpadDown(int x){ Remote(x); double POV = control.getPOV(); 
        if(POV >= 135 && POV < 225) { return true; } else { return false; }
    } public boolean DpadLeft(int x){ Remote(x); double POV = control.getPOV(); 
        if(POV >= 225 && POV < 315) { return true; } else { return false; }
    }
}