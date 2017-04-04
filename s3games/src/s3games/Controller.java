package s3games;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

import s3games.ai.*;
import s3games.engine.*;
import s3games.gui.*;
import s3games.io.*;
import s3games.player.*;
import s3games.robot.*;
import s3games.util.*;


/**
 * The Controller class is the main logic of the application. It holds one
 * ControllerWindow which is the main menu and responds to the user actions
 * by invoking the corresponding actions. 
 */
public class Controller implements SwitchListener, Runnable
{
    /** reference to the controller window - GUI with main menu */
    private ControllerWindow cw;
    
    /** reference to game window - it contains the game board visualizing the current game situation */
    private GameWindow gw;
    
    /** game logger is used to store log messages from the application to a file */
    private GameLogger logger;
    
    /** config is intended to store configuration options in a file with serialization, it is not in much use now */
    private Config config;
    
    /** instance of the game that is being currently played */
    private Game game;
    
    /** if the game terminates for whatever reason, this switch will be switched off, notifying all its listeners */
    private Switch gameRunning;
    
    /** singleton that communicates with the camera - i.e. a C++ application that utilizes OpenCV */
    private Camera camera;
    
    /** singleton that communicates with the Lynxmotion arm using SSC-32 controller connected to a serial port */
    private Robot robot;
    
    /** a synchronization object for notifying the controller thread that the game thread finished so that it can start another one (for the case of multiple games) */
    private final Object notifier;
    
    /** each game has an ID so that it can recognize it is the one that is still being played */
    private int currentGameID;

    /** constructs and shows main window and the minimum agenda */
    public Controller()
    {
        currentGameID = 1;
        notifier = new Object();
        gameRunning = new Switch();
        gameRunning.addSwitchListener(this);
        
        gw =  new GameWindow(gameRunning);
        cw  = new ControllerWindow(this);
        cw.setVisible(true);
        logger = new GameLogger();
        config = new Config(logger);
        config.load();
    }
    
    /** called automatically when a game starts or stops */
    @Override
    public void switchChanged(boolean newState)
    {
        if (newState == false)
        {
            cw.gameFinished(game.state.winner, game.state.playerScores);
            if (camera != null)
            {
                camera.close();
                camera = null;
            }
            synchronized(notifier)
            {
                notifier.notify();
            }            
        }
    }

    /** should return the list of available games, this should probably go to some external file (?) */
    public String[] getGameNames()
    {
        return new String[] {"MasterMind","Squares_v2","Squares","Connect4","Puzzle8","Skipping","Reversi","RiverCrossing","Alquerque","Mill","Nim", "Frogs", "TicTacToe"};
    }
    
    /** returns list of strategies that can play the specified game, we should improve this architecture somehow later */
    public String[] getStrategiesForGame(String gameName)
    {
        return Strategy.availableStrategies(gameName);
    }

    /** returns list of heuristics that are compatible with the specified strategy - also a place for improvement */
    public String[] getHeuristicsForGame(String strategyName) {
        return Heuristic.availableHeuristics(strategyName);
    }
    
    /** returns a list of strategies that can learn to play the specified game - improve...? */
    public String[] getLearnableStrategyTypesForGame(String gameName)
    {
        ArrayList<String> learnable = new ArrayList<String>();
        for(String s:getStrategiesForGame(gameName))
            if (Strategy.learnable(s))
                learnable.add(s);
        
        String[] result = new String[learnable.size()];
        return learnable.toArray(result);
    }

    /** return the number of players for the specified game - should read the specs...fixit! */
    public int getNumberOfPlayersForGame(String gameName)
    {
        return 2;
    }

    /** starts a single game
     *
     * @param gameName name of the game to play
     * @param boardType real or simulated
     * @param playerTypes for each player whether it is a human or a computer
     * @param playerStrategies only for computer players, name of strategy to use
     * @param strategyHeuristics name of heuristic to use for each computer player
     * @param numberOfRuns how many times to play this game (normally 1, but higher when running statistical runs)
     */
    public void play(String gameName, Player.boardType boardType, Player.playerType[] playerTypes, 
                     String[] playerStrategies, String[] strategyHeuristics, int numberOfRuns) 
    {   
        if (gameRunning.isOn()) return;
        
        gw.setVisible(true);
        
        gameSpecification = new GameSpecification(config, logger);
        try {
            gameSpecification.load(gameName);                        
        } catch (Exception e) { gw.showException(e); }

        this.numberOfRuns = numberOfRuns;
        this.boardType = boardType;
        this.playerTypes = playerTypes;
        this.playerStrategies = playerStrategies;
        this.strategyHeuristics = strategyHeuristics;
        new Thread(this).start();
    }

    /** the number of runs the controller thread should start in a sequence */
    private int numberOfRuns;
    
    /** the specification of the game being played */
    private GameSpecification gameSpecification;
    
    /** do we play real or on-screen? */
    private Player.boardType boardType;    
    
    /** do we play human/camera or computer/ai? */
    private Player.playerType[] playerTypes;
    
    /** strategy for each [computer] player */
    private String[] playerStrategies;
    
    /** heuristic for each [computer] player */
    private String[] strategyHeuristics;
    
    /** the name of game being played */
    private String gameName;
    
    /** a controller thread that plays all the games in this round - usually just one */
    @Override
    public void run()
    {
        FileWriter fw = null;
        try { fw = new FileWriter(cw.getStatisticFileName()); }
        catch (IOException ex) { gw.showException(ex); return; }
        
        while ((numberOfRuns-- > 0) && (gw.isVisible())) 
        {
            cw.setNumberOfRunsToGo(numberOfRuns+1);
            gameRunning.setValue(currentGameID++);
            gameRunning.on();            

            if (boardType == Player.boardType.REALWORLD)
            {
                camera = new Camera(gameSpecification);
                boolean robotNeeded = false;
                for (int player = 0; player < playerTypes.length; player++)
                    if (playerTypes[player] == Player.playerType.COMPUTER)
                        robotNeeded = true;
                try {
                    if (robotNeeded) robot = new Robot("/dev/ttyUSB0", gameSpecification);
                } catch (Exception e) { gw.showException(e); }
            }
                        
            game = new Game(config, logger, gw, gameRunning, robot);
            
            ArrayList<Player> players = new ArrayList<Player>();                
            for(int player = 0; player < gameSpecification.playerNames.length; player++)
            {
                Player p;
                if (playerTypes[player] == Player.playerType.HUMAN)
                {
                    if (boardType == Player.boardType.REALWORLD)
                        p = new CameraPlayer(gameSpecification, camera);
                    else p = new MousePlayer(gameSpecification, gw);
                }
                else 
                {
                    Heuristic h = Heuristic.getHeuristic(strategyHeuristics[player], gameSpecification);
                    p = Strategy.getStrategy(playerStrategies[player], h).getPlayer(gameSpecification);

                }
                p.setPlayerNumber(player+1);                
                p.setMaximumNumberOfNodes(cw.getNumberOfNodesToExpand());
                p.setMaxTime(cw.getRunningTime());
                players.add(p);
            }

            Player[] pls = new Player[players.size()];
            game.setGameAndPlayers(gameSpecification, players.toArray(pls));               
            synchronized(notifier)
            {
                game.start();
                try { notifier.wait(); } catch (InterruptedException ex) {}
            }
            cw.setNumberOfRunsToGo(0);
            try { fw.append(Integer.toString(game.state.winner) + System.getProperty("line.separator")); fw.flush(); }
            catch (IOException ex) { gw.showException(ex); }
        }
      
        try { fw.close(); } catch (IOException ex) { gw.showException(ex); }
    }
    
    /** tries to learn to play a game - not yet in use
     *
     * @param gameName name of the game to play
     * @param boardType real or simulated
     * @param playerTypes for each player whether it is a human or a computer
     * @param playerStrategies only for computer players, name of strategy to use
     * @param strategyHeuristics name of heuristic to use for computer players
     * @param strategyFileName where should we save the learned strategy
     * @param numberOfRuns how many times to play the game while learning
     * @param learnStrategyType strategy that will be learned
     * @return the number of player who won, 0 for draw/nobody, -1 if game was interrupted
     */
    public int learn(String gameName, Player.boardType boardType, Player.playerType[] playerTypes, 
                     String[] playerStrategies, String[] strategyHeuristics, String learnStrategyType, String strategyFileName, int numberOfRuns)
    {
        System.out.println(gameName);
        System.out.println(boardType);
        for (int i=0; i<playerTypes.length ; i++) {
           System.out.print(playerTypes[i]+" ");
           System.out.print(playerStrategies[i]+" ");
           System.out.println(strategyHeuristics[i]+" ");
         }
         System.out.println(learnStrategyType+" "+strategyFileName+" "+numberOfRuns);
        gw.setVisible(true);
        return 0;
    }    
    
    /** opens a window that allows a direct robot arm control - can be used for finding the robot arm angles of individual positions on the field */
    public void controlRobot() 
    {
        try {
            robot = new Robot("/dev/ttyUSB0", null);
            RobotControlWindow rcw = new RobotControlWindow(robot);        
        } catch (Exception e) {}
    }
}
