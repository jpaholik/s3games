package s3games.engine;

import java.util.*;
import s3games.gui.GameWindow;
import s3games.io.Config;
import s3games.io.GameLogger;
import s3games.player.Player;
import s3games.robot.Robot;
import s3games.util.Switch;

/** The Game class represents a single instance of game - it asks players to 
 * provide their moves, verifies if they conform with the rules and 
 * performs all the follow-up actions, including instructing the robot arm
 * to perform the requested actions */
public class Game extends Thread
{
    /** current state of the game */
    public GameState state;
    
    /** specification of the game */
    public GameSpecification gameSpecification;
    
    /** board of the game shown in window */
    public GameWindow window;
    
    /** config can contain some configuration options */
    public Config config;
    
    /** logger is used to write informative messages about game progress to file */
    public GameLogger logger;
    
    /** all players that are currently playing the game */
    public Player[] players;
    
    /** robot arm controller */
    private Robot robot;
    
    /** handle to a switch to turn off when the game terminates for any reason */
    private Switch gameRuns;
    
    /** current game ID */
    private int id;

    /** constructs a new game, saves references to the objects passed in arguments */
    public Game(Config config, GameLogger logger, GameWindow window, Switch gameRuns, Robot robot)
    {
        this.config = config;
        this.logger = logger;
        this.window = window;
        this.gameRuns = gameRuns;
        this.robot = robot;
        this.id = gameRuns.getValue();
    }

    /** setup game specification and players objects */
    public void setGameAndPlayers(GameSpecification gameSpecification, Player[] players)
    {        
        this.gameSpecification = gameSpecification;
        this.players = players;
    }
    
    /** game runs in separate thread, here */
    @Override
    public void run()  
    {
        try { 
        
        state = new GameState(gameSpecification, robot);
        int numberOfPlayers = gameSpecification.playerNames.length;
        window.setGame(this, gameRuns);
        
        do 
        {
            window.setState(state);
            Player playerOnMove = players[state.currentPlayer - 1];        
            ArrayList<Move> allowedMoves = state.allPossibleMoves();
            Move nextMove;
            
            //dbg
//            System.out.println("All possible moves:");
//            System.out.println(allowedMoves);
            
            //dbg 
            System.out.println("Different possible moves:");
            HashSet<Move> movesZ = new HashSet<Move>();
            movesZ.addAll(allowedMoves);  //similar as in state.possibleMoves method
            System.out.println(movesZ);
            
            if (allowedMoves.isEmpty())
            {
                state.winner = 0;
                state.touch();
                break;
            }
            
            boolean approved = false;
            do {
                nextMove = playerOnMove.move(state, allowedMoves);
                if (gameRuns.isOff() || (gameRuns.getValue() != id))
                {
                    state.winner = 0;
                    return;
                }
                System.out.println("Player moves: " + nextMove.toString());
                approved = state.moveAllowed(nextMove);
                if (!approved && !playerOnMove.isComputer())
                    window.showException(new Exception("You cannot make move " + nextMove + " according to rules. Try again."));
                
            } while (!approved && !playerOnMove.isComputer());
            
            if (!approved)
            {
                window.showException(new Exception("Player performed illegal move " + nextMove));
                state.winner = (state.currentPlayer % numberOfPlayers) + 1;
                state.touch();
                break;
            }
    
            if (robot != null)
                if (playerOnMove.isComputer())
                    robot.moveRobot(nextMove);
            
            state.performMove(nextMove);
            
            for (int p = 0; p < numberOfPlayers; p++)
                if (playerOnMove != players[p])
                    players[p].otherMoved(nextMove, state);            
            
        } while (state.winner == -1);
        
        window.setState(state);
        
        } catch (Exception e)
        {            
            e.printStackTrace();
            window.showException(e);
        }
        gameRuns.off();
    }
}
