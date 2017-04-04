package s3games.robot;

import java.io.*;
import java.util.Arrays;

import s3games.engine.GameSpecification;
import s3games.engine.Move;
import s3games.gui.RobotWindow;
import s3games.engine.Location;
import s3games.gui.RobotControlWindow;

/** The main robot arm controller. */
public class Robot implements Runnable
{
    /** serial port where the robot is connected, e.g. "COM3" */
    private String port;
    /** object that does the low-level communication with the SSC-32 (or whatever) servo-controller device */
    RobotSerialPort link;
    /** game specification of the game played */
    GameSpecification specs;
    /** reference to a robot control panel window */
    RobotWindow win;
    /** the move that is to be performed by the robot arm */
    Move requestedMove;
    /** synchronization object on which we wait until the move has been completed - before taking the next move */
    final Object waitForMovePerformed;
    
    /** construct the robot controller and connect to the servo controller device */
    public Robot(String serialPort, GameSpecification specs) throws Exception
    {
        waitForMovePerformed = new Object();
        port = serialPort;
        this.specs = specs;
        win = new RobotWindow(this);        
        open(); 
    }
    
    /** connect to the servo-controller device */
    private void open() throws Exception
    {
        win.addMessage("Opening robot...");
        link = new RobotSerialPort(port);
        link.open();       
        link.print(new RobotCmd(RobotCmd.Command.version).getCommand());
        StringBuilder sb = new StringBuilder();
        int ch;
        do {
            ch = link.read();
            sb.append((char)ch);
        } while (ch != 13);
        win.addMessage("Robot Firmware version: " + sb.toString());
        win.addMessage("You can move the robot away from the camera frame and click [Init] twice.");
    }
    
    /** close the communication link to the servo-controller device */
    public void close()
    {
        link.close();
    }
    
    /** perform a single move operation with the robot arm including picking and placing an element.
     * this method only requests that the move be performed and waits for confirmation. */
    public void moveRobot(Move move) throws Exception
    {
        requestedMove = move;
        win.addMessage("A new move " + move + " has been requested, click [Perform move] to perform it.");        
        synchronized(waitForMovePerformed)
        {
            waitForMovePerformed.wait();
        }
    }
 
    /** perform a single move that was requested transporting the element to be moved */
    public void performMove() throws Exception
    {
        goTo(requestedMove.from, true);        
        goTo(requestedMove.to, false);        
        goHome();
        synchronized(waitForMovePerformed)
        {
            waitForMovePerformed.notify();
        }
    }
    
    /** ask the servo-controller whether the robot has been performed */
    private boolean moveCompleted() throws Exception
    {
        link.print(new RobotCmd(RobotCmd.Command.query).getCommand());
        int stillMoving = link.read();
        if (stillMoving == RobotCmd.responseIdle()) return true;
        if (stillMoving == RobotCmd.responseBusy()) return false;
        throw new Exception("Query from robot expected + or . but " + stillMoving + " was received.");
    }

    /** the currently attempted arm configuration */
    private double[] currentLocation;
    /** move the robot arm to the specified location in two steps - first move above the location and then actually reaching it
     * @param locationName name of the location to be reached 
     * @param grab true indicates that we are to grab an element at this location, false indicates we are to release it there */
    private void goTo(String locationName, boolean grab) throws Exception
    {
        double[] angles = specs.locations.get(locationName).robot.angles;
        double[] coordinates = specs.locations.get(locationName).robot.coordinates;
        
        double[] place1 = Arrays.copyOfRange(angles, 0, 5);
        double[] place2 = Arrays.copyOfRange(angles, 5, 10);
        
        System.out.println("Angles upper pos: " + Arrays.toString(place1));
        System.out.println("Angles bottom pos: " + Arrays.toString(place2));
        System.out.println("Coordinates: " + Arrays.toString(coordinates));
        
        goTo(place1);
        Thread.sleep(700);
        currentLocation = place2;
        goTo(place2);
        Thread.sleep(700);
        if (grab) grab(); else put();
        if (paused) return;
        Thread.sleep(700);
        goTo(place1);
        Thread.sleep(700);
    }
    
    /** lets the user to enter a simple command for the robot on the standard 
     * input console - this is used only for testing the robot arm */
    public void doTest() 
    {
        System.out.print("enter angles (5 doubles, init, or home, grab, put): ");
        try {
            double angles[] = new double[5];
            BufferedReader in = new BufferedReader(new InputStreamReader(System.in));
            String[] ln = in.readLine().split(" ");
            if (ln.length == 0) return;
            if (ln[0].equals("init"))
            {
                System.out.println("Moving to initial position");
                initArm();
            }
            else if (ln[0].equals("home"))
            {
                System.out.println("Moving to home position");
                goHome();
            }
            else if (ln[0].equals("grab"))
            {
                System.out.println("Grabbing");
                grab();
            }
            else if (ln[0].equals("put"))
            {
                System.out.println("Putting");
                put();
            } else
            {
                if (ln.length < 5) System.out.println("Did anybody say 5 doubles?");
                else
                {
                    for (int i = 0; i < 5; i++) angles[i] = Double.parseDouble(ln[i]);
                    System.out.println("Moving the robot, watch out!");
                    Thread.sleep(1000);
                    goTo(angles);
                }
            }
        } catch (Exception e) {}
    }
    
    /** set to false to stop showing the locations demo */
    private boolean showingLocations;
    /** shows the user all locations learned by the robot arm, it does it 
     * in a separate thread of course as this method is called from the
     * button action listener */
    public void allLocationsDemo()
    {
        if (!showingLocations)
        {
            new Thread(this).start();
        }
    }
      
    /** performs the demo of showing all locations */
    @Override
    public void run() 
    {
        showingLocations = true;
        try {
            for (Location loc: specs.locations.values())
            {            
                if (loc.relevant == false)
                {
                  win.addMessage("skipped irrelevant location " + loc.name.fullName);
                  continue;
                }
                win.addMessage("goTo(" + loc.name.fullName + ") = " + loc.robot.toString());
                goTo(loc.name.fullName, false);
                Thread.sleep(5000);
                while (paused) Thread.sleep(1000);                    
            }
        } catch (Exception e) { win.addMessage("Problem moving to location: " + e.getMessage()); }
        showingLocations = false;
    }

    /** set to true to pause the show all locations demo */
    private boolean paused;
    
    /** invert the pause mode state */
    public void pause()
    {
        paused = !paused;
        if (paused) win.addMessage("Paused. Prese [Pause] again to continue.");
    }
    
    /** allows a direct control of the robot using a keyboard in a separate robot control window */
    public void control()
    {
        if (!paused) pause();
        win.addMessage("Control the robot using keys q,a,w,s,e,d,r,f,t,g, quit the window with 'Q'.");
        RobotControlWindow rcw = new RobotControlWindow(this);
        rcw.setPosition(currentLocation);
    }
    
    /** moves the robot arm to the requested angles configuration, if the previous move has been completed */
    public void goTo(double[] angles) throws Exception
    {
        while (!moveCompleted()) { Thread.sleep(500); }        
        link.print(new RobotCmd(RobotCmd.Command.position, angles).getCommand());
    }

    /** moves the robot arm to the requested angles configuration, regardless the previous move has been completed */
    public void goToDirect(double[] angles) throws Exception
    {    
        link.print(new RobotCmd(RobotCmd.Command.position, angles).getCommand());
    }
    
    /** grab the element now */
    public void grab() throws Exception
    {
        while (!moveCompleted()) { Thread.sleep(500); }
        link.print(new RobotCmd(RobotCmd.Command.grab).getCommand());
    }
   
    /** release the element now */
    public void put() throws Exception
    {
        while (!moveCompleted()) { Thread.sleep(500); }
        link.print(new RobotCmd(RobotCmd.Command.put).getCommand());        
    } 
   
    /** initialize the arm to its 0 position - a quite stressful operation! */
    public void initArm() 
    {
        try {
            link.print(new RobotCmd(RobotCmd.Command.init).getCommand());
        } catch (Exception e) { win.addMessage("Could not init robot arm: " + e.getMessage()); }
    }
     
    /** move the arm away from the sight of the camera */
    public void goHome() throws Exception
    {
        while (!moveCompleted()) { Thread.sleep(500); }
        link.print(new RobotCmd(RobotCmd.Command.home).getCommand());
    }

}
