package s3games.robot;

/** This class knows the details of the SSC-32 serial communication
 * protocol, and translates the meaningful commands into low-level
 * strings that are sent over the serial line to the SSC-32 servo controller */
public class RobotCmd 
{
    /** list of available commands */
    public enum Command { init, version, position, home, grab, put, query };
    /** the resulting command string */
    private String cmdString;

    /** construct a new command of the requested type - with arguments */
    public RobotCmd(Command type, double[] angles) 
    {
        switch (type) 
        {
            case position: buildPositionCommand(angles); break;
        }
    }

    /** construct a new command of the requested type - without arguments */
    public RobotCmd(Command type) 
    {
        switch (type) 
        {
            case version: cmdString = "VER"; break;
            case grab: cmdString = "#5 P1800 T2000"; break;
            case put: cmdString = "#5 P1000 T2000"; break;
            case query: cmdString = "Q"; break;
            case init: buildPositionCommand(new double[] {0.0, 0.0, 0.0, 0.0, 85.0 }); break;
            case home: buildPositionCommand(new double[] {0.0, 53.0, -76.0, -101.0, 13.0 }); break;
        }
    }

    /** a character sent as a response to the query command that indicates that the command has been completed */
    public static char responseIdle()
    {
        return '.';
    }
    
    /** a character sent as a response to the query command that indicates that the command is still being executed */
    public static char responseBusy()
    {
        return '+';
    }
    
    /** retrieve the command string */
    public String getCommand() 
    {
        return cmdString;
    }

    /** build a command for all servos from the angles */
    private void buildPositionCommand(double[] angles) 
    {
        StringBuilder result = new StringBuilder();
        result.append("#0 P");
        result.append(angleToPulseBase(angles[0]));
        result.append(" #1 P");
        result.append(angleToPulseWrist(angles[1]));
        result.append(" #2 P");
        result.append(angleToPulseElbow(angles[2]));
        result.append(" #3 P");
        result.append(angleToPulseShoulder(angles[3]));
        result.append(" #4 P");
        result.append(angleToPulseHand(angles[4]));
        result.append(" T3000");
        cmdString = result.toString();
    }
    
    /** translates the base angle to pulse */
    private int angleToPulseBase(double deg)
    {
        int r = (int) (1500 + (10 * deg));
        if (r > 2400) 
        {
            System.out.println("Pulse too long");
            r = 2400;
        }
        if (r < 600) 
        {
            System.out.println("Pulse too long");
            r = 600;
        }
        return r;        
    }

    /** translates the wrist angle to pulse */
    private int angleToPulseWrist(double deg) 
    {
        int r = (int) (1500 + (10 * deg));
        if (r > 2400) 
        {
            System.out.println("Pulse too long");
            r = 2400;
        }
        if (r < 600) 
        {
            System.out.println("Pulse too long");
            r = 600;
        }
        return r;
    }

    /** translates the elbow angle to pulse */
    private int angleToPulseElbow(double deg) 
    {
        int r = (int) (1800 - (10 * deg));
        if (r > 2400) 
        {
            System.out.println("Pulse too long");
            r = 2400;
        }
        if (r < 900) 
        {
            System.out.println("Pulse too long");
            r = 900;
        }
        return r;
    }

    /** translates the shoulder angle to pulse */
    private int angleToPulseShoulder(double deg) 
    {
        int r = (int) (1500 + (10 * deg));
        if (r > 2400) 
        {
            System.out.println("Pulse too long");
            r = 2400;
        }
        if (r < 600) 
        {
            System.out.println("Pulse too long");
            r = 600;
        }
        return r;
    }

    /** translates the hand angle to pulse */    
    private int angleToPulseHand(double deg)
    {
        int r = (int) (1500 + (10 * deg));
        if (r > 2400) 
        {
            System.out.println("Pulse too long");
            r = 2400;
        }
        if (r < 600) 
        {
            System.out.println("Pulse too long");
            r = 600;
        }
        return r;        
    }
}
