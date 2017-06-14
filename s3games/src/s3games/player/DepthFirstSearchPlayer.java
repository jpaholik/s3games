package s3games.player;

import java.util.*;
import s3games.engine.GameState;
import s3games.engine.Move;

/** Standard Depth-first-search player for one-player games. */
public class DepthFirstSearchPlayer extends Player 
{   
    /** list of open nodes (LIFO = stack) */
    private ArrayList<Node> open; 
    /** list of visited states */
    private HashSet<GameState> visited;
    /** history of states visited in previous calls to this search so that we 
     * do not generate looping moves */
    private HashSet<GameState> history;
    
    /** holds a state and previous node and move */
    class Node 
    {
        Move moveToThisState;
        GameState gs;
        Node previous;
        
        Node(Node p,Move m, GameState g) 
        {
            moveToThisState = m;
            previous = p;
            gs = g;
        }     
    }

    /** construct a DFS player for the specified game */
    public DepthFirstSearchPlayer() 
    {
        history = new HashSet<GameState>();
    }
    
    /** make a move using DFS strategy - search the state space in depth,
     * when finding a first winning state, return a move that is leading towards it */
    @Override
    public Move move(GameState state, ArrayList<Move> allowedMoves) throws Exception 
    {
        try { /*Thread.sleep(1000);*/ } catch (Exception e) {};
        open = new ArrayList<Node>();
        visited = new HashSet<GameState>();
        visited.addAll(history);
        
        GameState activeState;
        history.add(state.getCopy());
        visited.add(state);
        open.add(new Node(null, null, state));
               
        while (open.size() > 0) 
        {
                Node actualNode = open.get(open.size()-1);
                activeState = actualNode.gs;
                open.remove(open.size()-1);
                
                Set<Move> possibleMoves = activeState.possibleMoves();
                for (Move mv: possibleMoves) 
                {
                    GameState gs = activeState.getCopy( ); 
                    gs.performMove(mv); //perfom move and get next state
                    if ((!visited.contains(gs)) && (!history.contains(gs))) {
                        visited.add(gs);      //to prevent adding same neighbours
                        if (gs.winner == number)   //find a path to the first move
                        {
                            if (actualNode.previous == null) return mv;
                            while(actualNode.previous.previous!=null) 
                                   actualNode = actualNode.previous;
                            return actualNode.moveToThisState;
                        }
                        if (gs.winner == -1)   //not visited and not finished
                           open.add(new Node(actualNode,mv,gs));    
                    }
                }
        }        
        return allowedMoves.iterator().next();
    }
}
