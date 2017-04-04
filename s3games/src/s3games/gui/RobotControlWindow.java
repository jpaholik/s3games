package s3games.gui;

import java.awt.Color;
import java.awt.Graphics;
import javax.swing.JFrame;
import s3games.robot.Robot;

/** An auxiliary window that is used to control the robot arm with the keyboard. */
public class RobotControlWindow extends JFrame implements Runnable
{      
    /** handle to an object for interfacing with the robot */
    final Robot robot;
    
    /** list of keys we respond to */
    private final char keys[] = { 'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g' };
        
    /** current key detected */
    private char key;
    
    /** shall the window be closed? */
    private boolean terminate;
    
    /** current robot arm position */
    double[] position;
    
    /** update the current arm position and move the arm accordingly */
    public void respondToKey()
    {
        for (int i = 0; i < keys.length; i++)
           if (key == keys[i])
            {
                int degreeIndex = i / 2;
                int direction = ((i % 2) << 1) - 1;
                position[degreeIndex] += direction;
                repaint();
                try { robot.goToDirect(position); }
                catch (Exception e) {}                        
            }
    }
    
    /** reset the position to a specified one */
    public void setPosition(double[] pos)
    {
        position = pos;
    }
    
    /** construct and show the simple direct robot control window */
    public RobotControlWindow(Robot robotReference)
    {
        robot = robotReference;
        terminate = false;
        setTitle("Direct Robot Control");     
        position = new double[5];
        addKeyListener(new java.awt.event.KeyAdapter() {
            public void keyPressed(java.awt.event.KeyEvent evt) {
                key = evt.getKeyChar();   
                //System.out.println("listener: " + key);
                if (key == 'Q') terminate = true;
            }
        });
        setVisible(true);
        setFocusable(true);
        setSize(400,200);
        key = ' ';
        new Thread(this).start();
    }
    
    /** the window runs its own thread which responds to keys so that they 
     * do not fill the key buffer */
    @Override
    public void run()
    {
        while(!terminate)
        {
            
            if (key != ' ') 
            {
                respondToKey();
                key = ' ';
                try { Thread.sleep(250); } catch (Exception e) {}
            }
            
            try { Thread.sleep(250); } catch (Exception e) {}
        }        
        dispose();          
    }
    
    /** we let the user know the current robot configuration angles */
    @Override
    public void paint(Graphics g)
    {
        g.setColor(Color.white);
        g.fillRect(0, 0, this.getWidth(), this.getHeight());
        g.setColor(Color.magenta);
        StringBuilder s = new StringBuilder();
        if (position != null)
            for (int i = 0; i < 5; i++)
                s.append(' ').append(position[i]);
        g.drawString("Angles: " + s.toString(), 5, 5);
    }
}
