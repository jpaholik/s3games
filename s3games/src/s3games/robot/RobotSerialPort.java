package s3games.robot;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort; 

import java.io.*;
import java.util.Enumeration;

/** RobotSerialPort class is responsible for communicating with the SSC-32
 * robot controller over the serial line. It utilizes the RXTX library */
public class RobotSerialPort
{    
    /** name of the port, i.e. COM3 */
    private String port;
    
    /** the reading end of the serial port */
    private InputStream in;
    /** the writing end of the serial port */
    private OutputStream out;
    
    /** construct a robot serial port object with the specified port name */
    public RobotSerialPort(String portName)
    {
        port = portName;
    }
    
    /** open the communication link with the SSC-32 controller */
    public void open() throws Exception 
    {
        Enumeration e = CommPortIdentifier.getPortIdentifiers();
        while(e.hasMoreElements()) {
            System.out.println("e: "+ e.nextElement());
        }
        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(port);
        if (portIdentifier.isCurrentlyOwned())
            throw new Exception("RobotSerialPort: " + port + " is currently in use");
        
        CommPort commPort = portIdentifier.open(this.getClass().getName(),2000);
            
        if (commPort instanceof SerialPort) 
        {
                SerialPort serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(115200,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.FLOWCONTROL_NONE);
                
                in = serialPort.getInputStream();
                out = serialPort.getOutputStream();                
                if (in == null) throw new Exception("RobotSerialPort: cannot obtain input stream");
                if (out == null) throw new Exception("RobotSerialPort: cannot obtain output stream");
                         
        } else throw new Exception("RobotSerialPort: " + port + " is not a serial port.");
    }
    
    /** close the communication link with the SSC-32 controller */
    public void close() 
    {
        try 
        {
            in.close();
            out.close();
        } catch (Exception e) {}
    }

    /** send the packet to the SSC-32 */
    void print(String s) throws IOException
    {
        out.write(s.getBytes());
        out.write(0x0d);
        out.flush();
    }
    
    /** read a single byte from the SSC-32 */
    int read() throws IOException
    {
        int ch = -1;
        while ((ch == -1) || (ch == 255)) ch = in.read();
        return ch;
    }
}