/* this file contains classes for all the different lexeme types, they are
 * only used within this package, and thus with package visibility can 
 * reside in this file. */
package s3games.engine.expr;

import java.util.*;

/** All lexemes are derived from this general lexeme class */
public abstract class Lexeme
{    
    /** construct a list of the respective lexeme objects from the string holding an expression */
    public static ArrayList<Lexeme> parseLine(String ln) throws Exception
    {
        LexemeParser lp = new LexemeParser();
        return lp.parseLine(ln);
    }
}

/** represents a number lexeme */
class NumberLexeme extends Lexeme
{   
    /** the actual numeric value */
    int val;
    
    /** construct a number lexeme */
    NumberLexeme(int val)
    {
        this.val = val;
    }
}

/** represents a logic lexeme: true/false */
class BooleanLexeme extends Lexeme
{
    /** the actual logical value */
    boolean val;
    
    /** construct a logical lexeme */
    BooleanLexeme(boolean val)
    {
        this.val = val;
    }
}

/** represents a string lexeme */
class StringLexeme extends Lexeme
{
    /** the actual string value */
    String val;
    
    /** construct a string lexeme */
    StringLexeme(String str)
    {
        val = str;
    }
}

/** represents a set lexeme {a,b,...,z}, duplicity is allowed */
class SetLexeme extends Lexeme
{
    /** list of lexemes that form the set */
    ArrayList<Lexeme> elems;
    
    /** construct the set lexeme */
    SetLexeme(ArrayList<Lexeme> elements)
    {
        elems = elements;
    }
}

/** represents a lazy set lexeme {a,b,...,z}, which is used in "toLocations" attribute; duplicity is allowed */
class LazySetLexeme extends Lexeme
{
    /** list of lexemes that form the set */
    ArrayList<Lexeme> elems;
    
    /** construct the set lexeme */
    LazySetLexeme(ArrayList<Lexeme> elements)
    {
        elems = elements;
    }
}

/** represents a word lexeme - i.e. any string that was not recognized as anything else */
class WordLexeme extends Lexeme
{
    /** the actual string value */
    String val;
    
    /** construct a word lexeme */
    WordLexeme(String val)
    {
        this.val = val;
    }
}

/** represents a parenthesis lexeme (a,b,...,z) */
class ParenthesesLexeme extends Lexeme
{
    /** list of lexemes that form the list */
    ArrayList<Lexeme> lexs;
        
    /** construct a parenthesis lexeme */
    ParenthesesLexeme(ArrayList<Lexeme> lexemes)
    {
        lexs = lexemes;
    }
}

/** represents an operator lexeme - a word that was recognized to be an operator */
class OperatorLexeme extends Lexeme
{
    /** the operator type */
    Expr.operatorType op;
    
    /** construct an operator lexeme */
    OperatorLexeme(Expr.operatorType operator)
    {
        op = operator;
    }
}

/** represents an internal function lexeme - a word that was recognized to be an internal function */
class InternalFunctionLexeme extends Lexeme
{
    /** the internal function type */
    Expr.internalFunction fn;
    
    /** construct an internal function lexeme */
    InternalFunctionLexeme(Expr.internalFunction funct)
    {
        fn = funct;
    }
}

/** represents a variable lexeme $X */
class VariableLexeme extends Lexeme
{
    /** name of the variable without leading $ */
    String name;  
    
    /** construct the variable lexeme */
    VariableLexeme(String name)
    {
        this.name = name;
    }
}

/** represents a string with variable references - any string that has 
 * $VAR inside, however, variable name must be separated at the end by some 
 * lexeme separator, such as ',' ')', '(', '$', etc. */
class StringWithReferencesLexeme extends Lexeme
{
    /** original string from which the variables were removed, e.g. "el(,)" for "el($I,$J)" */
    String strWithoutVars; 
    
    /** list of variable references - keys are position in the above string, values are variable names */
    TreeMap<Integer, String> vars;
    
    /** construct a new expression with variable references */
    StringWithReferencesLexeme(String strWithoutVars, TreeMap<Integer, String> vars)
    {
        this.strWithoutVars = strWithoutVars;
        this.vars = vars;
    }
}