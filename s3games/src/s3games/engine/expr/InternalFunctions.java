package s3games.engine.expr;

import s3games.engine.*;
import s3games.util.IndexedName;

/** Holds implementations of all internal functions. The class is just 
 * a module with one long static eval() function that does the job depending
 * on which internal function is evaluated. */
public class InternalFunctions 
{
    /** Evaluates a specified internal function call with the specified arguments
     * in a given context. The evaluation can make changes in the context, and
     * game state and it can even move the robot arm, if needed. Whenever the
     * argument types do not match, or some other problem is detected, an
     * exception with the explanatory message is thrown. */
    static Expr eval(Expr.internalFunction fn, Expr[] args, Context context) throws Exception
    {
        if (fn == Expr.internalFunction.IF)
        {
            Expr cond = args[0].eval(context);
            if (!(cond instanceof Expr_LOG_CONSTANT))
                throw new Exception("IF with non-boolean condition");
            if (((Expr_LOG_CONSTANT)cond).b)
                return args[1].eval(context);
            else return args[2].eval(context);            
        }
        
        if ((fn == Expr.internalFunction.FORALL) ||
            (fn == Expr.internalFunction.FORSOME))
        {
            if (!(args[0] instanceof Expr_VARIABLE))
                throw new Exception("FORALL/FORSOME expects variable as its first argument");
            String varName = ((Expr_VARIABLE)args[0]).varName;
            Expr fromVal = args[1].eval(context);
            if (!(fromVal instanceof Expr_NUM_CONSTANT))
                throw new Exception("FORALL/FORSOME expects 'from' value to be a number");
            int fromValue = ((Expr_NUM_CONSTANT)fromVal).getInt();
            Expr toVal = args[2].eval(context);
            if (!(toVal instanceof Expr_NUM_CONSTANT))
                throw new Exception("FORALL/FORSOME expects 'to' value to be a number");            
            int toValue = ((Expr_NUM_CONSTANT)toVal).getInt();
            int delta = 1;
            if (fromValue > toValue) delta = -1;
            for (int i = fromValue; i != toValue + delta; i += delta )
            {
                context.setVar(varName, new Expr_NUM_CONSTANT(i));
                Expr val = args[3].eval(context);
                if (!(val instanceof Expr_LOG_CONSTANT))
                    throw new Exception("FORALL/FORSOME expects value to be true or false");
                if (fn == Expr.internalFunction.FORALL)
                {
                    if (!((Expr_LOG_CONSTANT)val).b)
                        return Expr.booleanExpr(false);
                }
                else if (((Expr_LOG_CONSTANT)val).b)
                    return Expr.booleanExpr(true);
                    
            }            
            return Expr.booleanExpr((fn == Expr.internalFunction.FORALL));
        }
        
        if (fn == Expr.internalFunction.LOCTYPE)
        {
            Expr locName = args[0].eval(context);
            if (!(locName instanceof Expr_STR_CONSTANT))
                throw new Exception("LOCTYPE requires string");
            String locationName = ((Expr_STR_CONSTANT)locName).str;
            Location location = context.specs.locations.get(locationName);
            if (location == null) return Expr.strExpr("");
            return Expr.strExpr(location.type);
        }
        
        if (fn == Expr.internalFunction.ELTYPE)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("ELTYPE requires string");
            String elementName = ((Expr_STR_CONSTANT)elName).str;
            Element element = context.specs.elements.get(elementName);
            if (element == null) return Expr.strExpr("");
            return Expr.strExpr(element.type);
        }
        
        if (fn == Expr.internalFunction.STATE)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("STATE requires string");
            String elementName = ((Expr_STR_CONSTANT)elName).str;
            Integer state = context.getState().elementStates.get(elementName);
            if (state == null) return Expr.numExpr(-1);
            return Expr.numExpr(state);
        }
        
        if (fn == Expr.internalFunction.LOCATION)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("LOCATION requires string");
            String elementName = ((Expr_STR_CONSTANT)elName).str;
            String location = context.getState().elementLocations.get(elementName);
            if (location == null) return Expr.strExpr("");
            return Expr.strExpr(location);
        }
        
        if (fn == Expr.internalFunction.CONTENT)
        {
            Expr locName = args[0].eval(context);
            if (!(locName instanceof Expr_STR_CONSTANT))
                throw new Exception("CONTENT requires string");
            String locationName = ((Expr_STR_CONSTANT)locName).str;            
            String content = context.getState().locationElements.get(locationName);
            if (content == null) return Expr.strExpr("");
            return Expr.strExpr(content);
        }
        
        if (fn == Expr.internalFunction.EMPTY)
        {
            Expr locName = args[0].eval(context);
            if (!(locName instanceof Expr_STR_CONSTANT))
                throw new Exception("CONTENT requires string");
            String locationName = ((Expr_STR_CONSTANT)locName).str;
            String content = context.getState().locationElements.get(locationName);
            return Expr.booleanExpr(content == null);
        }
        
        if (fn == Expr.internalFunction.INDEX)
        {
            Expr str = args[0].eval(context);
            if (!(str instanceof Expr_STR_CONSTANT))
                throw new Exception("INDEX requires string");
            IndexedName istr = new IndexedName(((Expr_STR_CONSTANT)str).str);
            if (istr.index.length < 1)
                throw new Exception("INDEX needs a string with at least one index");
            return Expr.numExpr(istr.index[0]);
        }
        
        if (fn == Expr.internalFunction.INDEXA)
        {
            Expr ind = args[0].eval(context);
            if (!(ind instanceof Expr_NUM_CONSTANT))
                throw new Exception("INDEXA requires number as the first argument");
            int index = ((Expr_NUM_CONSTANT)ind).getInt();
            Expr str = args[1].eval(context);
            if (!(str instanceof Expr_STR_CONSTANT))
                throw new Exception("INDEXA requires string as second argument");            
            IndexedName istr = new IndexedName(((Expr_STR_CONSTANT)str).str);
            if (istr.index.length < index)
                throw new Exception("INDEXA needs a string with " + index + " indexes");
            return Expr.numExpr(istr.index[index - 1]);        
        }
        
        if (fn == Expr.internalFunction.UNINDEX)
        {
            Expr str = args[0].eval(context);
            if (!(str instanceof Expr_STR_CONSTANT))
                throw new Exception("UNINDEX requires string");
            IndexedName istr = new IndexedName(((Expr_STR_CONSTANT)str).str);
            return Expr.strExpr(istr.baseName);        
        }
        
        if (fn == Expr.internalFunction.OWNER)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("OWNER requires string");
            String elementName = ((Expr_STR_CONSTANT)elName).str;
            Integer owner = context.getState().elementOwners.get(elementName);
            if (owner == null) return Expr.numExpr(-1);
            return Expr.numExpr(owner);            
        }
        
        if (fn == Expr.internalFunction.PLAYER)        
            return Expr.numExpr(context.getState().currentPlayer);
        
        if (fn == Expr.internalFunction.SCORE)
        {
            Expr player = args[0].eval(context);
            if (!(player instanceof Expr_NUM_CONSTANT))
                throw new Exception("SCORE requires player number");
            int p = ((Expr_NUM_CONSTANT)player).getInt();
            if ((p < 1 ) || (p > context.getState().playerScores.length))
                throw new Exception("unknown player " + p);
            return Expr.numExpr(context.getState().playerScores[p-1]);
        }
        
        if (fn == Expr.internalFunction.ZINDEX)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("ZINDEX requires string");
            String elementName = ((Expr_STR_CONSTANT)elName).str;
            Integer zindex = context.getState().elementzIndexes.get(elementName);            
            if (zindex == null) throw new Exception("ZINDEX(): unknown element " + elementName);
            return Expr.numExpr(zindex);
        }
        
        if (fn == Expr.internalFunction.MOVE)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("MOVE requires string as first argument");
            String elementName = ((Expr_STR_CONSTANT)elName).str;
            Element element = context.specs.elements.get(elementName);
            if (element == null) 
                throw new Exception("MOVE(): unknown element " + elementName);
            
            Expr locName1 = args[1].eval(context);
            if (!(locName1 instanceof Expr_STR_CONSTANT))
                throw new Exception("MOVE requires location name as second argument");
            String locationName1 = ((Expr_STR_CONSTANT)locName1).str;            
            Location location1 = context.specs.locations.get(locationName1);
            if (location1 == null) throw new Exception("MOVE(): unknown location " + locationName1);

            String currentLocation = context.getState().elementLocations.get(elementName);
            if (!currentLocation.equals(locationName1))
                throw new Exception("MOVE(): attempt to move element " + elementName + " from location " + locationName1 + " but its current location is " + currentLocation);
            
            Expr locName2 = args[2].eval(context);
            if (!(locName2 instanceof Expr_STR_CONSTANT))
                throw new Exception("MOVE requires location name as third argument");
            String locationName2 = ((Expr_STR_CONSTANT)locName2).str;            
            Location location2 = context.specs.locations.get(locationName2);
            if (location2 == null) throw new Exception("MOVE(): unknown location " + locationName2);
            if (context.getState().locationElements.get(locationName2) != null) return Expr.booleanExpr(false);
        
            Move mv = new Move(locationName1, locationName2, elementName, context.specs);
            context.getState().moveElement(mv, context.specs);
            if (context.robot != null) context.robot.moveRobot(mv);
            return Expr.booleanExpr(true);
        }
        
        if (fn == Expr.internalFunction.SETOWNER)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("SETOWNER requires string as first argument");
            String elementName = ((Expr_STR_CONSTANT)elName).str;
            Integer owner = context.getState().elementOwners.get(elementName);
            if (owner == null) throw new Exception("SETOWNER(): unknown element " + elementName);
            Expr ow = args[1].eval(context);
            if (!(ow instanceof Expr_NUM_CONSTANT))
                throw new Exception("SETOWNER requires number as second argument");
            int own = ((Expr_NUM_CONSTANT)ow).getInt();
            if ((own < 0 ) || (own > context.getState().playerScores.length))
                throw new Exception("SETOWNER requires existing player number or 0");
            context.getState().elementOwners.put(elementName, own);
            context.getState().touch();
            return Expr.booleanExpr(true);
        }
        
        if (fn == Expr.internalFunction.SETSTATE)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("SETSTATE requires string as first argument");
            String elementName = ((Expr_STR_CONSTANT)elName).str;            
            Element element = context.specs.elements.get(elementName);
            if (element == null) 
                throw new Exception("SETSTATE(): unknown element " + elementName);
            Expr st = args[1].eval(context);
            if (!(st instanceof Expr_NUM_CONSTANT))
                throw new Exception("SETSTATE requires number as second argument");
            int state = ((Expr_NUM_CONSTANT)st).getInt();
            if ((state < 1 ) || (state > context.specs.elementTypes.get(element.type).numStates))
                throw new Exception("SETSTATE on element " + elementName + " with state " + state + " is out of range");
            context.getState().elementStates.put(elementName, state);
            context.getState().touch();
            return Expr.booleanExpr(true);
        }
        
        if (fn == Expr.internalFunction.SETZINDEX)
        {
            Expr elName = args[0].eval(context);
            if (!(elName instanceof Expr_STR_CONSTANT))
                throw new Exception("SETZINDEX requires string as first argument");
            String elementName = ((Expr_STR_CONSTANT)elName).str;
            Integer zindex = context.getState().elementzIndexes.get(elementName);
            if (zindex == null) throw new Exception("SETZINDEX(): unknown element " + elementName);
            Expr zi = args[1].eval(context);
            if (!(zi instanceof Expr_NUM_CONSTANT))
                throw new Exception("SETZINDEX requires number as second argument");
            zindex = new Integer(((Expr_NUM_CONSTANT)zi).getInt());
            context.getState().elementzIndexes.put(elementName, zindex);  
            context.getState().touch();
            return Expr.booleanExpr(true);
        }
        
        if (fn == Expr.internalFunction.NEXTPLAYER)
        {
            context.getState().currentPlayer = (context.getState().currentPlayer % context.specs.playerNames.length) + 1;
            context.getState().touch();
            return Expr.booleanExpr(true);
        }
        
        throw new Exception("unknown internal function");
    }
}
