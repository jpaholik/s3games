package s3games.engine;

import java.util.*;
import s3games.engine.expr.Context;
import s3games.engine.expr.Expr;
import java.util.Arrays;
import s3games.robot.Robot;

/** GameState contains all information that is important for the current game state */
public class GameState
{
    /** for each element name, the state number */
    public Map<String,Integer> elementStates; // 1..numStates
    /** for each element name, location name where it currently is placed */
    public Map<String,String> elementLocations;
    /** for each location, content of the element */
    public Map<String,String> locationElements;
    /** for each element name, the number of player */
    public Map<String,Integer> elementOwners;
    /** the player number on move 1..N */
    public int currentPlayer;
    /** -1 until the game has finished, then 0 for draw, or 1..N player number who won */
    public int winner;

    /** a hash code for faster HashMap access */
    private int hash;
    /** true indicates that hash should be recomputed */
    private boolean modified;
    
    /** z-indexes of all the elements */
    public Map<String,Integer> elementzIndexes;
    
    /** scores of all players */
    public int[] playerScores;

    /** reference to the current context - this is not part of state, but needed for evaluation */
    private Context context;
        
    /** return a copy of this state state */
    public GameState getCopy()
    {
        GameState s = new GameState();
        s.elementzIndexes = new TreeMap<String,Integer>(elementzIndexes);
        s.playerScores = new int[playerScores.length];
        System.arraycopy(playerScores, 0, s.playerScores, 0, playerScores.length);
        s.elementStates = new TreeMap<String,Integer>(elementStates);
        s.elementLocations = new TreeMap<String,String>(elementLocations);
        s.locationElements = new TreeMap<String,String>(locationElements);
        s.elementOwners = new TreeMap<String,Integer>(elementOwners);
        s.currentPlayer = currentPlayer;
        s.winner = winner;
        s.hash = hash;
        s.modified = modified;
        s.context = context;
        return s;
    }
    
    @Override
    public String toString()
    {
        return hashString();
    }
    
    /** return a hash code for fast hashmap access, remember to call touch() always after state changes */
    @Override
    public int hashCode()
    {
        if (modified) recomputeHash();
        return hash;
    }
    
    private String hashString()
    {
        StringBuilder b = new StringBuilder();
        for (Map.Entry<String, String> loel: locationElements.entrySet())
        {
            if (!context.specs.locations.get(loel.getKey()).relevant) continue;
            b.append(loel.getKey());
            b.append('*');
            String el = loel.getValue();
            if (el != null)
            {
                b.append(context.specs.elements.get(el).type);
                b.append('*');
                b.append(elementStates.get(el));
                b.append('*');
                b.append(elementOwners.get(el));
                b.append('*');
                b.append(elementzIndexes.get(el));
            }
            b.append('*');
        }
        b.append(Integer.toString(currentPlayer));
        b.append('*');
        b.append(Integer.toString(winner));
        b.append('*');
        b.append(Arrays.toString(playerScores));
        return b.toString();
    }
    
    /** recomputes the hash code for this state using String.hasCode() */
    private void recomputeHash()
    {        
        hash = hashString().hashCode();        
    }
 
    /** always call this method after modifying the state */
    public void touch()
    {
        modified = true;
    }
    
    /** determine if the state is equal to another one - while ignoring irrelevant locations */
    @Override
    public boolean equals(Object other)
    {
        if (!(other instanceof GameState)) return false;
        return equals((GameState)other);
    }
        
    /** construct a new empty game state */
    public GameState()
    {
        init();
    }
    
    /** use this constructor to create a state corresponding to the start of game situation */
    public GameState(GameSpecification specs, Robot robot)
    {
        init();
        for (Map.Entry<String, Element> element: specs.elements.entrySet())
        {
            elementStates.put(element.getKey(), element.getValue().initialState);
            elementLocations.put(element.getKey(), element.getValue().initialLocation);
            locationElements.put(element.getValue().initialLocation, element.getKey());            
            elementOwners.put(element.getKey(), element.getValue().initialOwner);
        }
        for (Map.Entry<String,Element> element: specs.elements.entrySet())        
            elementzIndexes.put(element.getKey(), element.getValue().initialZindex);
        playerScores = new int[specs.playerNames.length];
        for (int i = 0; i < playerScores.length; i++) 
            playerScores[i] = specs.initialPlayerScore;
        context = new Context(this, specs, robot);
    }
    
    /** initialize all the structures */
    private void init()
    {
        elementStates = new TreeMap<String, Integer>();
        elementLocations = new TreeMap<String, String>();
        locationElements = new TreeMap<String, String>();
        elementOwners = new TreeMap<String, Integer>();
        elementzIndexes = new TreeMap<String, Integer>();        
        currentPlayer = 1;
        winner = -1;
        modified = true;
    }

    /** implementation of the equals() method - we ignore irrelevant locations and look only at element types, not the element names */
    public boolean equals(GameState other)
    {
        if (hashCode() != other.hashCode()) return false;
        
        Iterator<Map.Entry<String, String>> otherLocationElements = other.locationElements.entrySet().iterator();
        if (currentPlayer != other.currentPlayer) return false;
        for (Map.Entry<String, String> loel: locationElements.entrySet())
        {                        
            Map.Entry<String, String> otherloel = otherLocationElements.next();
            if (!context.specs.locations.get(loel.getKey()).relevant) continue;
            String el = loel.getValue();
            String otherel = otherloel.getValue();
            
            if (el != null)
            {          
                if (otherel == null) return false;
                if (!context.specs.elements.get(el).type.equals(context.specs.elements.get(otherel).type)) return false;
                if (!elementStates.get(el).equals(other.elementStates.get(otherel))) return false;
                if (!elementOwners.get(el).equals(other.elementOwners.get(otherel))) return false;
                if (!elementzIndexes.get(el).equals(other.elementzIndexes.get(otherel))) return false;
            }
            else if (otherel != null) return false;
        }        
        //if (!Arrays.equals(playerScores, other.playerScores)) return false;
        return true;
    }
        
    /** compares this state with newState, and returns a move that leads from this state to a new state */
    public Move findMove(GameState newState)
    {
        Move move = null;
        for(Map.Entry<String,String> eLoc: elementLocations.entrySet())
        {
            String element = eLoc.getKey();
            String location = eLoc.getValue();
            if (!location.equals(newState.elementLocations.get(element)))
            {
                if (move != null) return null;
                else move = new Move(location, newState.elementLocations.get(element), element, context.specs);
            }
        }
        return move;
    }
    
    /** return a list of moves that can be taken from this state - this includes ALL moves regardless that they are equal (because of irrelevance of locations or same element types) */
    public ArrayList<Move> allPossibleMoves() throws Exception
    {
        ArrayList<Move> moves = new ArrayList<Move>();
        context.setState(this);
        
        for (GameRule rule: context.specs.rules.values())        
            for (Element element: context.specs.elements.values())
            {
                ArrayList<Move> moreMoves = rule.getMatchingMoves(element, context.specs, context);
                if (moreMoves != null) moves.addAll(moreMoves);
            }
            return moves;
    }

    /** return a list of moves that can be taken from this states - this includes only DIFFERENT moves, i.e. those leading to not equal() states */
    public HashSet<Move> possibleMoves() throws Exception
    {
        Date getMovesStartTime = new Date();
        HashSet<Move> moves = new HashSet<Move>();
        moves.addAll(allPossibleMoves());
        Date getMovesEndTime = new Date();
        long duration = (getMovesEndTime.getTime() - getMovesStartTime.getTime());
        
        //System.out.println("Duration of getting all possible moves: "+ duration + "milisec");
        return moves;
    }
    
    /** verifies all game over conditions
     * @return the number of player who won 1..N, or 0 if end of game with draw, or -1 if not end of game */
    private int gameOver() throws Exception
    {
        context.setState(this);
        for (Map.Entry<Expr,Expr> cond: context.specs.terminationConditions.entrySet())
        {
            Expr evalResult = cond.getKey().eval(context);
            if (evalResult.isTrue())
                return cond.getValue().eval(context).getInt();        
        }
        return -1;
    }
    
    /** verifies all rules, returns true, if the move is allowed, or false if not,
     * does not modify the game state, does not make any followup actions, however
     * executes all conditions of tested rules with all the consequences */
    public boolean moveAllowed(Move move) throws Exception
    {
        context.setState(this);
        if (!elementLocations.get(move.element).equals(move.from))
            return false;
        if (locationElements.get(move.to) != null)
            return false;
        for (GameRule rule: context.specs.rules.values())        
            if (rule.matches(move, context)) return true;        
        return false;
    }

    /** performs a move after it has been verified, executes follow-up action
     * of the rule that maximizes the score, adds the score, important: the
     * game is marked finished only if one of the players won, or rules 
     * specifically announced draw. when there are no more moves, you need
     * to determine draw state on your own */
    public void performMove(Move move) throws Exception
    {      
        context.setState(this);
        GameRule bestRule = findBestRule(move);
        if (bestRule==null) throw new Exception("Trying to perform a move that is not legal in this state " + move);
        bestRule.addScores(context);
        moveElement(move, context.specs);
        
        if(move.action != null) {
            move.action.eval(context);
        }
        else {
            bestRule.performAction(context);
        }
        updateScores(context);
        winner = gameOver();
        modified = true;
    }
    
    /** updates scores after a move has been performed including the follow-up action
     * based on the current state, and the context (including variables) of that move */
    private void updateScores(Context context) throws Exception
    {
        for (GameScoring scoring: context.specs.scorings)
            if (scoring.situation.eval(context).isTrue())
                for (int i = 0; i < scoring.players.size(); i++)
                {
                    int player = scoring.players.get(i).eval(context).getInt();
                    int amount = scoring.amounts.get(i).eval(context).getInt();
                    context.getState().playerScores[player - 1] += amount;
                }
    }
    
    /** finds the rule that maximizes the score for the specified move  */
    private GameRule findBestRule(Move move) throws Exception
    {
        int maximumScoreGained = Integer.MIN_VALUE;
        GameRule bestRule = null;
        for (GameRule rule: context.specs.rules.values())        
            if (rule.matches(move, context))
            {
                if (bestRule == null) bestRule = rule;            
                for(int i = 0; i < rule.scorePlayer.size(); i++)
                    if (rule.scorePlayer.get(i).eval(context).getInt() == currentPlayer)
                    {
                        int score = rule.scoreAmount.get(i).eval(context).getInt();
                        if (score > maximumScoreGained)
                        {
                            maximumScoreGained = score;
                            bestRule = rule;
                        }
                    }                  
            }
        return bestRule;
    }
    
    /** only updates the game state by moving element between two locations, does not test anything, does not apply any rules */
    public void moveElement(Move move, GameSpecification specs)
    {
        elementLocations.put(move.element, move.to);
        locationElements.put(move.from, null);
        locationElements.put(move.to, move.element);
        modified = true;
    }
}
