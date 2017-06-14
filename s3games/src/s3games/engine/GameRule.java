package s3games.engine;

import java.util.ArrayList;
import s3games.engine.expr.Context;
import s3games.engine.expr.Expr;

/** The GameRule class represents a single rule for game moves: from where,
 * to where, and what can be moved, and under what circumstances, as well
 * as what has to be done as follow-up action and how to adjust the player
 * scores, if the rule was applied. The names of element and locations can
 * contain variables - then these are instantiated with the respective indexes
 * of the element/location where the rule is applied. For example a rule
 *   from: loc($X1,$Y1); to: loc($X2,$Y2), element: el($I)
 * will match with any locations named "loc(any_number,any_number)" and
 * any element with name "el(any_number)", and the variables will be filled
 * with the respective indexes for the condition, score, and followup evaluation
 * contexts */
public class GameRule
{
    /** name of the rule */
    public String name;
    /** element to move - expression that evaluates to string or a string with variable reference */
    public Expr element;
    /** elementType to move - expression that evaluates to string or a string with variable reference */
    public Expr elementType;
    /** state of the element - or an expression returning a number */
    public Expr state;
    /** expression evaluating to a number of the player that can make this move */
    public Expr currentPlayer;
    /** location from where the element can be moved - evaluates to string or a string with variable references */
    public Expr from;
    /** location to where the element can be moved - evaluates to string or a string with variable references */
    public Expr to;
    /** locations to where the element can be moved - evaluates to ArrayList of strings or strings with variable references */
    public Expr toLocations;
    /** an expression that must evaluate to true in order for this rule to be applicable */
    public Expr condition;
    /** a list of players for whom the score should be updated, if the rule was applied */
    public ArrayList<Expr> scorePlayer;
    /** a list of score values to add to each player that is in scorePlayer list in the same index */
    public ArrayList<Expr> scoreAmount;
    /** action expression to evaluate, after the rule was applied */
    public Expr action;

    /** construct an empty rule with the specified name */
    public GameRule(String name)
    {
        this.name = name;
        scorePlayer = new ArrayList<Expr>();
        scoreAmount = new ArrayList<Expr>(); 
        condition = Expr.booleanExpr(true);
    }

    /** verifies whether the specified move conforms to this rule in the provided context */
    public boolean matches(Move move, Context context) throws Exception
    {        
        GameState st = context.getState();
        if (element.matches(move.element, context))
            if ((state == null) || (state.matches(st.elementStates.get(move.element), context)))
                if ((currentPlayer == null) || (currentPlayer.matches(st.currentPlayer, context)))
                    if (from.matches(move.from, context))
                        if (to.matches(move.to, context))                    
                            return condition.eval(context).isTrue();
        return false;
    }
        
    /** list of moves that can be performed from this state with the element specified in the first argument
     * @return the list or null, if no such moves exist */
    public ArrayList<Move> getMatchingMoves(Element el, GameSpecification specs, Context context) throws Exception
    {
        GameState st = context.getState();
        ArrayList<Move> moves = new ArrayList<Move>();
        if (element.matches(el.name.fullName, context))
            if ((state == null) || (state.matches(st.elementStates.get(el.name.fullName), context)))
                if ((currentPlayer == null) || (currentPlayer.matches(st.currentPlayer, context)))
                {
                    String tryFrom = st.elementLocations.get(el.name.fullName);
                    if (from.matches(tryFrom, context))
                        if (toLocations != null) {
                            Expr exp = toLocations.eval(context);
                            
                            // list of possible moves from tryFrom location
                            ArrayList<Expr> expList = exp.getSet();
                            
                            if(!expList.isEmpty()) {
                                for(Expr e: expList) {
                                        String toPos;
                                        
                                        // try to get followup action for this move
                                        try {
                                            ArrayList<Expr> moveDetails = e.eval(context).getSet();
                                            
                                            // on zero position there should be name of TO place
                                            toPos = moveDetails.get(0).eval(context).getStr();
                                            // on first position there should be followup action
                                            Expr action = moveDetails.get(1);
                                            
                                            moves.add(new Move(tryFrom, toPos, el.name.fullName, specs, action));
                                        }
                                        // if there is no followup action for this move
                                        catch (Exception exc) {
                                            toPos = e.eval(context).getStr();
                                            moves.add(new Move(tryFrom, toPos, el.name.fullName, specs));
                                        }
                                }
                            }
                        }
                        else {
                            for (Location tryTo:specs.locations.values())
                                if (st.locationElements.get(tryTo.name.fullName) == null)
                                    if (to.matches(tryTo.name.fullName, context))
                                        if (condition.eval(context).isTrue())
                                            moves.add(new Move(tryFrom, tryTo.name.fullName, el.name.fullName, specs));
                        }
                }
        if (moves.size() > 0) return moves;
        return null;
    }
    
    /** modify the player scores after this rule was matched */
    public void addScores(Context context) throws Exception
    {
        GameState gs = context.getState();
        
        for(int i = 0; i < scorePlayer.size(); i++)
        {
            int player = scorePlayer.get(i).eval(context).getInt();
            int amount = scoreAmount.get(i).eval(context).getInt();
            gs.playerScores[player-1] += amount;
        }
    }
    
    /** perform the follow-up action, after this rule was matched */
    public void performAction(Context context) throws Exception
    {
        if (action != null) action.eval(context);
    }
}
