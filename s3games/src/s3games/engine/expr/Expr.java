package s3games.engine.expr;

import java.util.*;
import s3games.util.IndexedName;

/** Abstract class for all expression types. Contains the enumerations of all 
 * internal operators and functions. */
public abstract class Expr
{
    /** list of all supported internal operators */
    public enum operatorType { EQUALS, NOTEQUALS, LOWER, LOWEREQUAL, GREATER, GREATEREQUAL,
                        PLUS, MINUS, TIMES, DIV, MOD, ABS, SUBSET, ELEMENT, SETMINUS,
                        UNION, INTERSECTION, AND, OR, NOT, ASSIGNMENT, UNKNOWN };

    /** list of all supported internal functions */
    public enum internalFunction { IF, FORALL, FORSOME, LOCTYPE, ELTYPE, STATE, LOCATION,
                             CONTENT, EMPTY, INDEX, INDEXA, UNINDEX, OWNER, PLAYER,
                             SCORE, ZINDEX, MOVE, SETOWNER, SETSTATE, SETZINDEX, NEXTPLAYER, UNKNOWN }

    /** Allows appending a new expression to this expression, 
     * thus getting a Expr_LIST if it was not already. 
     * It is used for representing multi-line named expressions. */
    public Expr append(Expr expr)
    {
        Expr el = this;
        if (!(el instanceof Expr_LIST)) el = new Expr_LIST(this);
        ((Expr_LIST)el).add(expr);
        return el;
    }
    
    /** Parse the expression from a string representation as appears in the 
     * game specification file. First, the expression is broken into a list
     * of lexemes, which are then parsed into internal expression representation */
    public static Expr parseExpr(String ln) throws Exception
    {
        ArrayList<Lexeme> lexs = Lexeme.parseLine(ln);
        return ExprParser.parseExpr(lexs);
    }
                
    /** convert an operator lexeme to the respective operator */
    public static operatorType getOperatorType(String op)
    {
        if (op.equals("==")) return operatorType.EQUALS;
        else if (op.equals("!=")) return operatorType.NOTEQUALS;
        else if (op.equals("<")) return operatorType.LOWER;
        else if (op.equals("<=")) return operatorType.LOWEREQUAL;
        else if (op.equals(">")) return operatorType.GREATER;
        else if (op.equals(">=")) return operatorType.GREATEREQUAL;
        else if (op.equals("+")) return operatorType.PLUS;
        else if (op.equals("-")) return operatorType.MINUS;
        else if (op.equals("*")) return operatorType.TIMES;
        else if (op.equals("/")) return operatorType.DIV;
        else if (op.equals("%")) return operatorType.MOD;
        else if (op.equals("ABS")) return operatorType.ABS;
        else if (op.equals("SUBSET")) return operatorType.SUBSET;
        else if (op.equals("IN")) return operatorType.ELEMENT;
        else if (op.equals("SETMINUS")) return operatorType.SETMINUS;
        else if (op.equals("UNION")) return operatorType.UNION;
        else if (op.equals("INTERSECT")) return operatorType.INTERSECTION;
        else if (op.equals("AND")) return operatorType.AND;
        else if (op.equals("OR")) return operatorType.OR;
        else if (op.equals("NOT")) return operatorType.NOT;
        else if (op.equals("=")) return operatorType.ASSIGNMENT;
        else return operatorType.UNKNOWN;
    }

    /* convert an internal function lexeme to the respective internal function */
    public static internalFunction getInternalFunction(String fn)
    {
        if (fn.equals("IF")) return internalFunction.IF;
        else if (fn.equals("FORALL")) return internalFunction.FORALL;
        else if (fn.equals("FORSOME")) return internalFunction.FORSOME;
        else if (fn.equals("LOCTYPE")) return internalFunction.LOCTYPE;
        else if (fn.equals("ELTYPE")) return internalFunction.ELTYPE;
        else if (fn.equals("STATE")) return internalFunction.STATE;
        else if (fn.equals("LOCATION")) return internalFunction.LOCATION;
        else if (fn.equals("CONTENT")) return internalFunction.CONTENT;
        else if (fn.equals("EMPTY")) return internalFunction.EMPTY;
        else if (fn.equals("INDEX")) return internalFunction.INDEX;
        else if (fn.equals("INDEXA")) return internalFunction.INDEXA;
        else if (fn.equals("UNINDEX")) return internalFunction.UNINDEX;
        else if (fn.equals("OWNER")) return internalFunction.OWNER;
        else if (fn.equals("PLAYER")) return internalFunction.PLAYER;
        else if (fn.equals("SCORE")) return internalFunction.SCORE;
        else if (fn.equals("ZINDEX")) return internalFunction.ZINDEX;
        else if (fn.equals("MOVE")) return internalFunction.MOVE;
        else if (fn.equals("SETOWNER")) return internalFunction.SETOWNER;
        else if (fn.equals("SETSTATE")) return internalFunction.SETSTATE;
        else if (fn.equals("SETZINDEX")) return internalFunction.SETZINDEX;
        else if (fn.equals("NEXTPLAYER")) return internalFunction.NEXTPLAYER;
        else return internalFunction.UNKNOWN;
    }

    /** constructor method - returns a numerical expression that holds the specified number */
    public static Expr numExpr(int num)
    {
        return new Expr_NUM_CONSTANT(num);
    }
    
    /** constructor method - returns a string expression that holds the specified string */
    public static Expr strExpr(String str)
    {
        return new Expr_STR_CONSTANT(str);
    }
    
    /** constructor method - returns a boolean expression that holds the specified logical value */
    public static Expr booleanExpr(boolean b)
    {
        return new Expr_LOG_CONSTANT(b);
    }
    
    /** stub to be overriden by expressions that are of numeric type, otherwise throws exception */
    public int getInt() throws Exception
    {
        throw new Exception("expected numeric expression, but it's " + this + " here");
    }
    
    /** stub to be overriden by expressions that are of string type, otherwise throws exception */
    public String getStr() throws Exception
    {
        throw new Exception("expected string expression, but it's " + this + " here");        
    }
    
    /** stub to be overriden by expressions that are of boolean type, otherwise returns false */
    public boolean isTrue()
    {
        return false;
    }
    
    /** stub to be overriden by expressions that are of boolean type, otherwise returns false
     * note: here we do not use closed-world assumption: anything else than false is not false! */
    public boolean isFalse()
    {
        return false;
    }
    
    /** stub to be overriden by expressions that are of boolean type, otherwise returns false
     * note: here we do not use closed-world assumption: anything else than false is not false! */
    public ArrayList<Expr> getSet() throws Exception
    {
        throw new Exception("expected set expression, but it's " + this + " here");
    }
        
    /** stub to be overriden by expressions that are of string type - it should 
     * set the variables in the context, if the expression contains variable references */
    public boolean matches(String s, Context context)
    {
        return false;
    }
    
    /** stub to be overriden by expressions that are of numeric type - it should 
     * set the variables in the context, if the expression is a variable */
    public boolean matches(int i, Context context)
    {
        return false;
    }
    
    /** stub to be overriden by expressions that are of string type - it should 
     * set the variables in the context, if the expression contains variable references */  
    public boolean matches(IndexedName name, Context context)
    {
        return false;
    }
    
    /** evaluates this expression and returns the resulting value */
    public abstract Expr eval(Context context) throws Exception;
    
    /** force evaluation of this expression, even of LazySet, and returns the resulting value */
    public Expr forceEval(Context context) throws Exception {
        return this.eval(context);
    }
    
    /** compares if the values of this expression and the other expression are the same */
    public boolean equals(Expr other, Context context) throws Exception
    {
        Expr val = eval(context);
        return val.equals(other, context);
    }
}

