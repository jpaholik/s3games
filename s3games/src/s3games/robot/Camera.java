package s3games.robot;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.ArrayList;

import s3games.engine.GameSpecification;
import s3games.engine.Location;
import s3games.gui.CameraWindow;

/** Camera singleton is an object that communicates with the C++ application
 * to retrieve the locations of the elements on the real board. The communication
 * is done through a redirected standard input/output of the C++ application. */
public class Camera implements Runnable
{
    /** represents an object that has been detected by the camera */
    public class DetectedObject
    {
        /** element type */
        public String type;
        /** element state */
        public int state;
        /** location in the camera image: x-coordinate */
        public int x;
        /** location in the camera image: y-coordinate */        
        public int y;

        /** construct a new detected object */
        public DetectedObject(String type, int state, int x, int y)
        {
            this.type = type;
            this.state = state;
            this.x = x;
            this.y = y;
        }

        /** visualize the detected object - for printing/debugging purposes */
        @Override
        public String toString()
        {        
            return type + "(" + state + ") " + x + " " + y;
        }
    }
    
    /** reference to the current game specification */
    GameSpecification specs;    
    
    /** reference to the camera control window */
    CameraWindow win;
    
    /** the communication thread for talking with the C++ application */
    Thread commThread;
    
    /** shall the debugging messages from C++ application (D:XXXX) be shown? */
    boolean showDebuggingMessages;
    
    /** the list of the detected objects */
    ArrayList<DetectedObject> objects;
    
    /** the input link from the C++ application */
    BufferedReader in;
    /** the output link to the C++ application */
    PrintWriter out;
    /** shall we close all camera agenda? */
    boolean terminating;
    /** we synchronize with the camera player about making a new move through this object */
    final Object notificator;

    /** the name of the C++ application to be started */
    private static final String CAMERA_PROGRAM = "/home/krtek/bakalarka/source/s3games/s3games/cameraBoard.exe";
    
    /** construct a new camera controller, it will also construct a camera control window. */
    public Camera(GameSpecification specs)
    {
        notificator = new Object();
        objects = new ArrayList<DetectedObject>();
        showDebuggingMessages = false;
        terminating = false;
        this.specs = specs;
        win = new CameraWindow(this);
        commThread = new Thread(this);
        init();        
    }
        
    /** sends the color/size parameters of element types and states to the C++ application */
    private void sendObjectParametersToCamera()
    {
        ArrayList<CameraObjectType> params = specs.cameraObjectTypes;
        out.println("2");
        out.println(params.size());
        for (CameraObjectType o: params)                    
            o.printTo(out);        
        out.flush();
    }
    
    /** sends all the location coordinates to the camera for the purpose of the 
     * visualization - pressing 'l' key in the camera window shows them - in this
     * way the designer of the real game board can check if she got the pixel
     * coordinates properly. */
    private void sendLocationsToCamera()
    {
        out.println("3");
        
        int count = 0;
        for (Location loc: specs.locations.values())    
            if (loc.camera != null) count++;
        out.println(count);
        
        for (Location loc: specs.locations.values())    
            if (loc.camera != null)
                out.println(loc.camera.x + " " + loc.camera.y);
        
        out.flush();
    }
    
    /** launch the camera C++ application, send all the configuration
     * information and start the communication in a separate thread */
    private void init()
    {
        try 
        {
            win.addMessage("Connecting to camera...");
            ProcessBuilder pb = new ProcessBuilder(CAMERA_PROGRAM);
            Process p = pb.start();
            in = new BufferedReader(new InputStreamReader(p.getInputStream()));
            out = new PrintWriter(p.getOutputStream());
            int terminated = 0;
            boolean hasTerminated = true;
            try { terminated = p.exitValue(); } 
            catch (IllegalThreadStateException e) { hasTerminated = false; }
            if (hasTerminated) throw new Exception("The camera program has terminated with exit code " + terminated);
            String header = in.readLine();
            if (header == null) throw new Exception("Could not start talking with the camera program");
            if (!header.equals("S:S3 Games Camera"))
                throw new Exception("Camera program did not respond properly");
            win.addMessage("Camera there. Sending object types...");
            sendObjectParametersToCamera();
            sendLocationsToCamera();
            commThread.start();
            Thread.sleep(700);
        } 
        catch (Exception e) { win.addMessage(e.getMessage()); }
        
    }
    
    /** the communicating thread - blocking reads from the C++ application
     * and processes the received packets */
    @Override
    public void run()
    {
        try {
            do {
                String ln = in.readLine();                
                if (ln == null) throw new Exception("Camera disconnected");
                if (ln.charAt(0) == 'F') throw new Exception("Camera program stoped with error " + ln.substring(2));
                else if (ln.charAt(0) == 'D') 
                {
                    if (showDebuggingMessages) win.addMessage(ln.substring(2));
                    else continue;
                }
                else if (ln.charAt(0) == 'I') win.addMessage(ln.substring(2));
                else if (ln.charAt(0) == 'O') addObject(ln.substring(2));
                else if (ln.charAt(0) == '=') objectsTransmitted();
            } while (!terminating);
            
            win.close();            
        } catch (Exception e)
        {
            if (!terminating) win.addMessage(e.getMessage());
            e.printStackTrace();
        }
        win = null;
    }
    
    /** add a new object detected by the camera application to the list of detected objects */
    private void addObject(String objDesc) throws Exception
    {
        String obj[] = objDesc.split("\t");
        if (obj.length != 4) throw new Exception("Camera program output error");
        objects.add(new DetectedObject(obj[0], Integer.parseInt(obj[1]), Integer.parseInt(obj[2]), Integer.parseInt(obj[3])));
    }
    
    /** indicates that the camera application has finished sending the detected locations - the list is now complete */
    private void objectsTransmitted()
    {
        //win.addMessage(objects.toString());
        win.addMessage("Received " + objects.size() + " objects from camera.");
        synchronized(notificator) { notificator.notify(); }
    }
    
    /** sends a command to the C++ application to make a snapshot 
     * of the current game situation and send a list of detected objects */
    public void requestObjectsFromCamera()
    {
        objects.clear();
        out.println("1");
        out.flush();
    }
    
    /** the camera player indicates to the camera object by calling this blocking
     * method that it is interested in obtaining the current state of the game
     * from the camera immediately */
    public ArrayList<DetectedObject> waitForUserMove() 
    {
        win.addMessage("It is your turn: move one stone and click the button below.");
        try { synchronized(notificator) {             
            notificator.wait(); 
        } } catch (Exception e) {}
        return objects;
    }    
    
    /** appends a new message to the camera control panel window - a gateway for camera player */
    public void msgToUser(String msg)
    {
        win.addMessage(msg);
    }

    /** close all the camera-related stuff */
    public void close()
    {
        terminating = true;
        if (out != null) 
        {
            out.println("0");
            out.flush();
        }        
        out.close();
        try { 
            Thread.sleep(2000);
            in.close(); 
        } catch (Exception e) {}
    }
}
