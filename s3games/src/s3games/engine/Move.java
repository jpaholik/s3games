package s3games.engine;
import s3games.engine.expr.Expr;

/** Represents a single move made by some player: what was moved, from where, to where */
public class Move
{
    /** name of the element that was moved */
    public final String element; 
    /** name of the location from where the element was moved  */
    public final String from;
    /** name of the location to where the element was moved */
    public final String to; 
    /** action expression to evaluate, after the move was performed */
    public Expr action;
    
    /** remembers the hashstring of this move so that it does not need to be reconstructed */
    private final String hashString;
    /** remembers the hash value of this move */
    private final int hash;    
    
    public Move(String from, String to, String element, GameSpecification specs)
    {
        this.from = from;
        this.to = to;
        this.element = element;        
        hashString = computeHashString(specs);
        hash = hashString.hashCode();
    }
    
    public Move(String from, String to, String element, GameSpecification specs, Expr action)
    {
        this.from = from;
        this.to = to;
        this.element = element;      
        this.action = action;
        hashString = computeHashString(specs);
        hash = hashString.hashCode();
    }
    
    /** computes the hash string for this move for comparison of equality - takes into account location relevance and element types */
    private String computeHashString(GameSpecification specs)
    {
        LocationType fromType = specs.locationTypes.get(specs.locations.get(from).type);
        LocationType toType = specs.locationTypes.get(specs.locations.get(to).type);
        String fromHash = from;
        String toHash = to;
        String elementType = specs.elements.get(element).type;   
        String elementState = Integer.toString(specs.elements.get(element).initialState);
        if (!fromType.relevant) fromHash = fromType.name;
        if (!toType.relevant) toHash = toType.name;        
       // StringBuilder s = new StringBuilder(fromHash.length() + toHash.length() + elementType.length() + 4);
        StringBuilder s = new StringBuilder(fromHash.length() + toHash.length() + elementType.length() + elementState.length()+ 6);
        
        s.append(fromHash);
        s.append('*');
        s.append(toHash);
        s.append('*');
        s.append(elementType);
        s.append('*');
        s.append(elementState);
        return s.toString();
    }
    
    /** for printing and debugging */
    @Override
    public String toString()
    {
        return from + "->" + to + ":" + element;
    }
    
    /** return a hash code for fast hashmap access */
    @Override
    public int hashCode()
    {                
        return hash;
    }
    
    /** compare for equality taking into account element types and location relevance, i.e. whether the moves are really different */
    @Override
    public boolean equals(Object other)
    {        
        if (other instanceof Move)
        {            
            Move otherMove = (Move)other;
            if (hash != otherMove.hash) return false;
            if (hashString.equals(otherMove.hashString)) return true;
        }
        return false;
    }
}
