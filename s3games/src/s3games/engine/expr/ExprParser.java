package s3games.engine.expr;

import java.util.ArrayList;
import java.util.Arrays;

/** A parser that takes a list of lexemes and constructs an internal 
 * representation of the expression. The class contains only static methods
 * and no fields, it is just a module to keep this code in. */
public class ExprParser 
{
    /** Parses an infix operator with two arguments. */
    private static Expr parseOperator(ArrayList<Lexeme> leftArgument, ArrayList<Lexeme> rightArgument, Expr.operatorType op) throws Exception
    {
        Expr firstArgument = parseExpr(leftArgument);
        Expr secondArgument = parseExpr(rightArgument);
        return new Expr_OPERATOR(op, new Expr[] { firstArgument, secondArgument });            
    }
        
    /* Parses a list of expressions, returns an array of the parsed expressions. 
     * These are typically comma-separated expressions provided as arguments to 
     * a function call, or members of some set.  */
    private static Expr[] parseList(ArrayList<Lexeme> argLexs) throws Exception
    {
        Expr[] listOfArgs = new Expr[argLexs.size()];
        ArrayList<Lexeme> oneArg = new ArrayList<Lexeme>();
        for (int i = 0; i < listOfArgs.length; i++)                    
        {
            if (i > 0) oneArg.clear();
            oneArg.add(argLexs.get(i));
            listOfArgs[i] = parseExpr(oneArg);
        }
        return listOfArgs;
    }
    
    /** parse the expression from the provided list of lexemes */
    static Expr parseExpr(ArrayList<Lexeme> lexs) throws Exception
    {        
        if (lexs.isEmpty()) throw new Exception("empty expression");
        Lexeme lex = lexs.get(0);
        lexs.remove(0);
        if (lex instanceof OperatorLexeme)
        {
            Expr.operatorType oper = ((OperatorLexeme)lex).op;
            if ((oper == Expr.operatorType.NOT) ||
                (oper == Expr.operatorType.ABS) ||
                (oper == Expr.operatorType.MINUS))
            {
                if (lexs.isEmpty()) throw new Exception("operator NOT, ABS or MINUS without argument");
                return new Expr_OPERATOR(oper, new Expr[] { parseExpr(lexs) });
            }            
            throw new Exception("misplaced operator");
        }
            
        if (!lexs.isEmpty())
        {
            Lexeme lex2 = lexs.get(0);           
                
            if (lex2 instanceof OperatorLexeme)
            {
                lexs.remove(0);
                ArrayList<Lexeme> first = new ArrayList<Lexeme>();
                first.add(lex);
                return parseOperator(first, lexs, ((OperatorLexeme)lex2).op);
            }
        }
        
        if (lexs.size() > 1)
        {
            Lexeme lex2 = lexs.get(0);
            Lexeme lex3 = lexs.get(1);

            if ( ((lex instanceof WordLexeme) || (lex instanceof InternalFunctionLexeme)) &&
                 (lex2 instanceof ParenthesesLexeme) && (lex3 instanceof OperatorLexeme) )
            {                
                lexs.remove(0);
                lexs.remove(0);
                ArrayList<Lexeme> first = new ArrayList<Lexeme>();
                first.add(lex);
                first.add(lex2);
                return parseOperator(first, lexs, ((OperatorLexeme)lex3).op);
            }
        }
        
        if (!lexs.isEmpty())
        {
            Lexeme lex2 = lexs.get(0);
            if (((lex instanceof WordLexeme) || (lex instanceof InternalFunctionLexeme)) &&
                (lex2 instanceof ParenthesesLexeme))
            {
                lexs.remove(0);
                Expr[] listOfArgs = parseList(((ParenthesesLexeme)lex2).lexs);
                if (lex instanceof WordLexeme)
                    return new Expr_EXPRESSION_CALL(((WordLexeme)lex).val, listOfArgs);
                else return new Expr_INTERNAL_FN(((InternalFunctionLexeme)lex).fn, listOfArgs);
            }
        }
                
        if (!lexs.isEmpty()) 
            throw new Exception("unexpected trailing lexemes " + lexs);
        if (lex instanceof NumberLexeme)
            return new Expr_NUM_CONSTANT(((NumberLexeme)lex).val);
        if (lex instanceof BooleanLexeme)
            return new Expr_LOG_CONSTANT(((BooleanLexeme)lex).val);
        if (lex instanceof StringLexeme)
            return new Expr_STR_CONSTANT(((StringLexeme)lex).val);
        if (lex instanceof WordLexeme)
            return new Expr_EXPRESSION_CALL(((WordLexeme)lex).val, new Expr[0]);
        if (lex instanceof InternalFunctionLexeme)
            return new Expr_INTERNAL_FN(((InternalFunctionLexeme)lex).fn, new Expr[0]);
        if (lex instanceof SetLexeme)
            return new Expr_SET(new ArrayList<Expr>(Arrays.asList((parseList(((SetLexeme)lex).elems)))));
        if (lex instanceof LazySetLexeme)
            return new Expr_LAZYSET(new ArrayList<Expr>(Arrays.asList((parseList(((LazySetLexeme)lex).elems)))));
        if (lex instanceof VariableLexeme)
            return new Expr_VARIABLE(((VariableLexeme)lex).name);
        if (lex instanceof StringWithReferencesLexeme)
            return new Expr_STRING_WITH_VAR_REF(((StringWithReferencesLexeme)lex).strWithoutVars, 
                                                ((StringWithReferencesLexeme)lex).vars);
        if (lex instanceof ParenthesesLexeme) 
            return parseExpr(((ParenthesesLexeme)lex).lexs);
        throw new Exception("unexpected lexeme" + lex);     
    }
}
