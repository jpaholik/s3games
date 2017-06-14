package s3games.engine.expr;

import java.util.*;

/** The LexemeParser class breaks a single line string into a list of lexemes */
class LexemeParser 
{
    /** here are all the lexeme separators that indicate locations where the previous lexeme terminates */
    private static final String allSeparators = "=!<=>+-*/% \t(){}[],\"$";    
    
    /** resulting pre-lexemes in raw string format before they are recognized */
    private ArrayList<String> ln;
    
    /** for matching parentheses and braces */
    private LinkedList<Character> stack;

    /** parse a single line with expression to a list of lexemes. 
     * only splits the line to pre-lexemes and calls parseLn() to
     * do the actual work. */
    ArrayList<Lexeme> parseLine(String line) throws Exception
    {        
        StringTokenizer st = new StringTokenizer(line, allSeparators, true);
        ln = new ArrayList<String>();
        while (st.hasMoreElements()) 
        {
            String token = st.nextToken();
            if ((!token.equals(" ")) && (!token.equals("\t")))
                ln.add(token);
        }            

        stack = new LinkedList<Character>();
        return parseLn();
    }
    
    /** determine the respective closing parenthesis */
    private char closingBracketFor(String bracket)
    {
        if (bracket.charAt(0) == '(') return ')';
        else if (bracket.charAt(0) == '[') return ']';
        return '}';
    }

    /** parses a list of raw pre-lexemes into valid lexemes */
    private ArrayList<Lexeme> parseLn() throws Exception
    {
        ArrayList<Lexeme> acc = new ArrayList<Lexeme>();
        while (!ln.isEmpty())
        {
            String token = ln.get(0);
            ln.remove(0);
            
            if (token.equals("$")) 
            {
                if (ln.isEmpty()) throw new Exception("$ without name");
                String varName = ln.get(0);
                ln.remove(0);
                acc.add(new VariableLexeme(varName));
                continue;
            }
            if (token.equals("!"))
            {
                if (ln.isEmpty()) throw new Exception("unexpected terminating !");
                token += ln.get(0);
                ln.remove(0);
            }
            else if (token.equals("<") || token.equals(">") || token.equals("="))
            {
                if (ln.isEmpty()) throw new Exception("unexpected terminating " + token);
                if (ln.get(0).equals("="))
                {
                    token += "=";
                    ln.remove(0);
                }
            }
            Expr.operatorType opType = Expr.getOperatorType(token);
            if (opType != Expr.operatorType.UNKNOWN)
            {
                acc.add(new OperatorLexeme(opType));
                continue;
            }
            Expr.internalFunction fn = Expr.getInternalFunction(token);
            if (fn != Expr.internalFunction.UNKNOWN)
            {
                acc.add(new InternalFunctionLexeme(fn));
                continue;
            }
            if (token.equals("\""))
            {
                TreeMap<Integer,String> vars = new TreeMap<Integer, String>();
                StringBuilder sb = new StringBuilder();
                do
                {
                    if (ln.isEmpty()) throw new Exception("parsing expression: malformed string");
                    token = ln.get(0);   
                    ln.remove(0);
                    if (token.equals("\"")) 
                        break;
                    if (token.equals("$"))
                    {                        
                        if (ln.isEmpty()) throw new Exception("parsing expression: malformed var ref. inside string");
                        vars.put(sb.length(), ln.get(0));
                        ln.remove(0);
                    }
                    else sb.append(token);                    
                } while (true);
                if (vars.size() > 0)
                    acc.add(new StringWithReferencesLexeme(sb.toString(), vars));
                else acc.add(new StringLexeme(sb.toString()));
                continue;
            }
            if (token.toLowerCase().equals("true"))
            {
                acc.add(new BooleanLexeme(true));
                continue;
            }
            if (token.toLowerCase().equals("false"))
            {
                acc.add(new BooleanLexeme(false));
                continue;
            }
            if (Character.isDigit(token.charAt(0)))
            {
                acc.add(new NumberLexeme(Integer.parseInt(token)));
                continue;
            }
            if (token.equals("{") || token.equals("(") || token.equals("["))
            {                
                int previousStackSize = stack.size();
                stack.push(closingBracketFor(token));
                ArrayList<Lexeme> elems = new ArrayList<Lexeme>();
                while (stack.size() > previousStackSize)
                {
                    ArrayList<Lexeme> elems1 = parseLn();
                    if (stack.size() > previousStackSize) 
                        stack.pop(); // ','
                    if(!elems1.isEmpty()) {
                        elems.add(new ParenthesesLexeme(elems1));                    
                    }
                }
                if (token.equals("{"))
                    acc.add(new SetLexeme(elems));
                else if (token.equals("["))
                    acc.add(new LazySetLexeme(elems));
                else
                    acc.add(new ParenthesesLexeme(elems));
                continue;
            }
            if (token.equals(","))
            {
                if (stack.isEmpty()) throw new Exception("misplaced comma");
                stack.push(',');
                return acc;                
            }
            if (token.equals("}") || token.equals(")") || token.equals("]"))
            {
                if (!stack.isEmpty()) 
                    if (stack.pop().equals(token.charAt(0)))                
                        return acc;
                throw new Exception("misplaced terminating bracket " + token);
            }
            acc.add(new WordLexeme(token));
        }
        if (!stack.isEmpty()) throw new Exception("missing terminating bracket " + stack.pop());
        return acc;
    }
}
