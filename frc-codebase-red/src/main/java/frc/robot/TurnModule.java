package frc.robot;
import edu.wpi.first.wpilibj.Notifier;

public class TurnModule{
    private Notifier TurningPID; 
    private double error, diffError, lastError, testPIDOutput, navTo; 
    private volatile double PIDOutput = 0;
    static final double kP = 1, kD = .1; 
    public TurnModule() {
    	lastError = getError(); 
    	TurningPID = new Notifier(() ->  {
    		error = getError(); 
    		diffError = lastError - error; 
            testPIDOutput = kP * error + kD * diffError; 
            testPIDOutput = Math.min(testPIDOutput, .8);
            PIDOutput = Math.max(testPIDOutput, -.8); 
            lastError = error; 
    	}); 
    	TurningPID.startPeriodic(0.05);
    }
    
    public double getError(){double navFinal = boundHalfDegrees(navTo)/180; return navFinal;}	
	public static double boundHalfDegrees(double angle){while(angle>=180)angle-=360; while(angle<-180)angle+=360; return angle;}
    public void setYaw(double degree){navTo = degree;}
    public double getPIDOutput(){try {return PIDOutput;} catch (Exception e) {return 0; }}
}
