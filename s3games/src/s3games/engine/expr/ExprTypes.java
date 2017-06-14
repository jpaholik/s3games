/* this file contains all the specific expression types 
 * for the internal expression representation. They all have
 * only package visibility since only the abstract Expr needs
 * to be visible from outside, thus they can all live together
 * in this file. */
package s3games.engine.expr;

import java.util.ArrayList;
import java.util.Map;
import java.util.TreeMap;

/** a numerical constant expression */
class Expr_NUM_CONSTANT extends Expr
{
    /** the actual numeric value */
    private int num;

    /** construct a numeric expression */
    public Expr_NUM_CONSTANT(int num)
    {
        this.num = num;
    }

    /** evaluating a number gives that number */
    @Override
    public Expr eval(Context context)
    {        
        return this;
    }
    
    /** numeric expressions are equal, if they have the same number */
    @Override
    public boolean matches(int i, Context context)
    {
        return num == i;
    }
    
    /** get the actual number */
    @Override
    public int getInt()
    {
        return num;
    }
    
    /** compare this number to another expression - it must evaluate to the same number */
    @Override
    public boolean equals(Expr other, Context context) throws Exception
    {
        other = other.eval(context);
        if (other instanceof Expr_NUM_CONSTANT)        
            return num == ((Expr_NUM_CONSTANT)other).num;
        return false;
    }
}

/** a string constant expression */
class Expr_STR_CONSTANT extends Expr
{
    /** the actual string */
    public String str;

    /** create a new string expression */
    public Expr_STR_CONSTANT(String str)
    {
        this.str = str;
    }

    /** string evaluates to string */
    @Override
    public Expr eval(Context context)
    {
        return this;
    }
    
    /** string matches another string if they are equal */
    @Override
    public boolean matches(String str, Context context)
    {
        return this.str.equals(str);
    }
    
    /** get the actual string */
    @Override
    public String getStr()
    {
        return str;
    }
    
    /** does the string match another expression? - must evaluate to string that is equal, 
     * i.e. matching of string with variable ref. can be done only left-to-right */
    @Override
    public boolean equals(Expr other, Context context) throws Exception
    {
        other = other.eval(context);
        if (other instanceof Expr_STR_CONSTANT)        
            return str.equals(((Expr_STR_CONSTANT)other).str);
        return false;
    }
}

/** represents an expression that is a logical constant */
class Expr_LOG_CONSTANT extends Expr
{
    /** the actual boolean value */
    public boolean b;

    /** construct a new logical constant */
    public Expr_LOG_CONSTANT(boolean b)
    {
        this.b = b;
    }

    /** logical constant evaluates to logical constant */
    @Override
    public Expr eval(Context context)
    {
        return this;
    }
    
    /** check if the value is true */
    @Override
    public boolean isTrue()
    {
        return b;
    }
    
    /** check if the value is false */
    @Override
    public boolean isFalse()
    {
        return !b;
    }
    
    /** logical constant is equal to expression that evaluates to the same logical constant */
    @Override
    public boolean equals(Expr other, Context context) throws Exception
    {
        other = other.eval(context);
        if (other instanceof Expr_LOG_CONSTANT)        
            return b == ((Expr_LOG_CONSTANT)other).b;
        return false;
    }
}

/** Represents a set of expressions */
class Expr_SET extends Expr
{
    /** the actual set */
    public ArrayList<Expr> set;

    /** construct a set of expressions */
    public Expr_SET(ArrayList<Expr> contents)
    {
        set = contents;
    }

    /** a set evaluates to a new set of expressions that are all evaluated to 
     * their values, duplicity is not checked here - perhaps it should be? */
    @Override
    public Expr eval(Context context) throws Exception
    {
        ArrayList<Expr> newContents = new ArrayList<Expr>();
        for(Expr e: set)
            newContents.add(e.eval(context));
        return new Expr_SET(newContents);
    }

    /** the set of expression is equal to another one if they contain expressions 
     * that evaluate to the same values (recursively). Must have the same length,
     * but duplicity is not checked - perhaps it should be? */
    @Override
    public boolean equals(Expr other, Context context) throws Exception
    {
        Expr elems[] = new Expr[set.size()];
        for(int i = 0; i < set.size(); i++)
            elems[i] = set.get(i).eval(context);        
        other = other.eval(context);
        if (other instanceof Expr_SET)
        {
            if (elems.length != ((Expr_SET)other).set.size())
                return false;
            for (int i = 0; i < set.size(); i++)
                if (!elems[i].equals(((Expr_SET)other).set.get(i), context))
                    return false;
        }
        else return false;
        return true;
    }
    
    @Override
    /** return the expression set */
    public ArrayList<Expr> getSet() throws Exception
    {
        
        return set;
    }
}

/** Represents a set of expressions */
class Expr_LAZYSET extends Expr
{
    /** the actual set */
    public ArrayList<Expr> set;

    /** construct a set of expressions */
    public Expr_LAZYSET(ArrayList<Expr> contents)
    {
        set = contents;
    }

    /** a set evaluates to a new set of expressions that are all evaluated to 
     * their values, except expressions of type Expr_EXPRESSION_CALL; 
     * duplicity is not checked here - perhaps it should be? */
    @Override
    public Expr eval(Context context) throws Exception
    {
        ArrayList<Expr> newContents = new ArrayList<Expr>();
        for(Expr e: set) {
            if(e instanceof Expr_EXPRESSION_CALL) {
                Expr[] args = ((Expr_EXPRESSION_CALL)e).args;
                Expr[] evalArgs = new Expr[args.length];
                
                for(int j = 0; j < args.length; j++) {
                    evalArgs[j] = args[j].eval(context);
                }
                
                newContents.add(new Expr_EXPRESSION_CALL(((Expr_EXPRESSION_CALL)e).exprName, evalArgs));                        
            }
            else {
                newContents.add(e.eval(context));
            }
        }
        return new Expr_LAZYSET(newContents);
    }
    
    /** a set evaluates to a new set of expressions that are all evaluated to 
     * their values, duplicity is not checked here - perhaps it should be? */
    @Override
    public Expr forceEval(Context context) throws Exception
    {
        ArrayList<Expr> newContents = new ArrayList<Expr>();
        for(Expr e: set)
            newContents.add(e.eval(context));
        return new Expr_LAZYSET(newContents);
    }

    /** the set of expression is equal to another one if they contain expressions 
     * that evaluate to the same values (recursively). Must have the same length,
     * but duplicity is not checked - perhaps it should be? */
    @Override
    public boolean equals(Expr other, Context context) throws Exception
    {
        Expr elems[] = new Expr[set.size()];
        for(int i = 0; i < set.size(); i++) {
            Expr e = set.get(i);
            
            if(e instanceof Expr_EXPRESSION_CALL) {
                Expr[] args = ((Expr_EXPRESSION_CALL)e).args;
                Expr[] evalArgs = new Expr[args.length];
                
                for(int j = 0; j < args.length; j++) {
                    evalArgs[j] = args[j].eval(context);
                }
                
                elems[i] = new Expr_EXPRESSION_CALL(((Expr_EXPRESSION_CALL)e).exprName, evalArgs);
            }
            else {
                elems[i] = set.get(i).eval(context);
            }
        }
        
        other = other.eval(context);
        if (other instanceof Expr_LAZYSET)
        {
            if (elems.length != ((Expr_LAZYSET)other).set.size())
                return false;
            
            for (int i = 0; i < set.size(); i++) {
                Expr e1 = elems[i];
                Expr e2 = ((Expr_LAZYSET) other).set.get(i);
                
                if(e1 instanceof Expr_EXPRESSION_CALL) {
                    if(e2 instanceof Expr_EXPRESSION_CALL) {
                        Expr_EXPRESSION_CALL exc1 = ((Expr_EXPRESSION_CALL) e1);
                        Expr_EXPRESSION_CALL exc2 = ((Expr_EXPRESSION_CALL) e2);
                        
                        if(!exc1.exprName.equals(exc2.exprName)) 
                            return false;
                        
                        if(exc1.args.length != exc2.args.length) 
                            return false;
                        
                        for(int j = 0; j < exc1.args.length; j++) {
                            if(exc1.args[j].equals(exc2.args[j], context))
                                return false;
                        }
                    }
                    else {
                        return false;
                    }
                }
                else {
                    if(e2 instanceof Expr_EXPRESSION_CALL) {
                        return false;
                    }
                    else {
                        if (!elems[i].equals(((Expr_LAZYSET)other).set.get(i), context))
                            return false;
                    }
                }
                
            }
        }
        else return false;
        return true;
    }
    
    @Override
    /** return the expression set */
    public ArrayList<Expr> getSet() throws Exception
    {
        return set;
    }
}

/* Represents a variable expression, such as $X - the $ sign is not part 
 * of the varName internally, only in game specification file */
class Expr_VARIABLE extends Expr
{
    /** the name of the variable, such as A for the variable $A */
    String varName;   

    /** create new variable expression */
    public Expr_VARIABLE(String var)
    {
        varName = var;
    }

    /** evaluation will evaluate the value of the variable in the context
     * provided or throw exception if it is not present */
    @Override
    public Expr eval(Context context) throws Exception
    {
        Expr val = context.getVar(varName);
        if (val == null) throw new Exception("evaluating unbound variable " + varName);
        return val;
    }
    
    /** variable matches a string always, this set its value in the context */
    @Override
    public boolean matches(String str, Context context)
    {
        context.setVar(varName, new Expr_STR_CONSTANT(str));
        return true;
    }
    
    /** variable matches a number always, this sets its value in the context */
    @Override
    public boolean matches(int i, Context context)
    {
        context.setVar(varName, new Expr_NUM_CONSTANT(i));
        return true;
    }
        
}

/** Represents a string that contains variable references, for instance "element($X,$Y)" */
class Expr_STRING_WITH_VAR_REF extends Expr
{
    /** string from which or variables were cut out, for instance "el(,)" for "el($X,$Y)" */
    String str;
    /** a map of all variable references in the resulting string after cutting, keys are
     * the indexes in the string after variable removals, values() are the names of the variables 
     * without $ sign, for instance (3,X),(4,Y) for "el($X,$Y)". */
    TreeMap<Integer, String> varRefs;    

    /** construct a new expression with variable references */
    public Expr_STRING_WITH_VAR_REF(String str, TreeMap<Integer, String> varRefs)
    {
        this.str = str;
        this.varRefs = varRefs;
    }

    /** string with variable references evaluates to string taking the values of the variables from the context */
    @Override
    public Expr eval(Context context) throws Exception
    {
        StringBuilder s = new StringBuilder("");
        int pos = 0;
        for(Map.Entry<Integer, String> var: varRefs.entrySet())
        {
            s.append(str.substring(pos, var.getKey()));
            s.append(context.getVar(var.getValue()).getInt());
            pos = var.getKey();
        }
        s.append(str.substring(pos));
        return new Expr_STR_CONSTANT(s.toString());
    }
    
    /** when matching string with variable references to another string, we will 
     * set the values of the refered variables, if the two strings can be matched */
    @Override
    public boolean matches(String s, Context context)
    {
        String[] base = s.split("[0-9]+");
        String[] numbers = s.split("[^0-9]+");
        StringBuilder glued = new StringBuilder();
        if (numbers.length - 1 != varRefs.size()) return false;
        for(String b:base) glued.append(b);        
        if (!glued.toString().equals(str)) return false;
        int pos = 0;
        int[] positions = new int[numbers.length - 1];
        for(int i = 0; i < numbers.length - 1; i++)
        {
            pos += base[i].length();
            positions[i] = pos;
        }
        int i = 0;
        for (Integer p: varRefs.keySet())
            if (p != positions[i++]) return false;
        i = 0;
        for (String v: varRefs.values())
            context.setVar(v, new Expr_NUM_CONSTANT(Integer.parseInt(numbers[++i])));
        return true;
    }
}

/** Represents an expression that is a user-defined expression call 
 * (with 0 or more expressions to be supplied in named arguments) */
class Expr_EXPRESSION_CALL extends Expr
{
    /** name of the expression to call */
    String exprName;   
    /** the arguments provided */
    Expr[] args;

    /** construct a new expression call */
    public Expr_EXPRESSION_CALL(String exprName, Expr[] args)
    {
        this.exprName = exprName;
        this.args = args;
    }

    /** evaluate the arguments of this expression and assign the resulting values
     * to variables of the formal argument names of the expression called 
     * @param context - the context where the variables will be created
     * @param argNames - the list of argument names - the names of the variables where the values of the argument expressions will be assigned */
    void evalArgs(Context context, String[] argNames) throws Exception
    {        
        if (args.length != argNames.length) 
            throw new Exception("function expects " + argNames.length + " arguments, but " + args.length + " provided");
        for (int i = 0; i < args.length; i++)
        {
            Expr evaluatedArg = args[i].eval(context);
            context.setVar(argNames[i], evaluatedArg);
        }
    }
    
    /** expression call evaluation will evaluate the body of the called 
     * expression with the specified arguments after they are evaluated
     * from left-to-right. The variables of the argument names are 
     * not cleared after the expression evaluation 
     * is finished, they will remain in the current context. */
    @Override
    public Expr eval(Context context) throws Exception
    {
        Expression expr = context.getExpr(exprName);
        if (expr == null) throw new Exception("Attempt to call a non-existing function " + exprName);
        evalArgs(context, expr.argNames);
        return expr.expr.eval(context);
    }
}

/** Represents a call of an internal function with a set of expressions to be 
 * supplied in the arguments */
class Expr_INTERNAL_FN extends Expr
{
    /** which internal function to call */
    Expr.internalFunction fn;
    
    /** list of expressions values of which are sent in internal fn. arguments */
    Expr[] args;

    /** construct an internal function */
    public Expr_INTERNAL_FN(Expr.internalFunction fn, Expr[] args)
    {
        this.fn = fn;
        this.args = args;
    }

    /** internal function evaluation will call the respective internal function,
     * the arguments are not evaluated here, since the internal function can
     * treat the order of evaluation differently or selectively, such as IF-THEN-ELSE */
    @Override
    public Expr eval(Context context) throws Exception
    {
        return InternalFunctions.eval(fn, args, context);
    }
}

/** Represents an internal operator expression with a set of expressions to be
 * supplied in the arguments */
class Expr_OPERATOR extends Expr
{
    /** which internal operator to call */
    Expr.operatorType op;
    
    /** list of expressions values of which are sent in internal op. arguments */
    Expr[] args;

    /** construct a new internal operator */
    public Expr_OPERATOR(Expr.operatorType op, Expr[] args)
    {
        this.op = op;
        this.args = args;
    }
    
    /** internal operator evaluation will call the respective internal operator,
     * the arguments are not evaluated here, since the internal operator can
     * treat the order of evaluation differently or selectively, such as AND */
    @Override
    public Expr eval(Context context) throws Exception
    {
        return Operators.eval(op, args, context);
    }
}

/** Represents a list of expressions - in the game spec. file this is the
 * case of multi-line named expression */
class Expr_LIST extends Expr
{
    /** the list of the individual independent expressions */
    ArrayList<Expr> exprs;
    
    /** construct a list of expressions */
    public Expr_LIST(Expr expr)
    {
        exprs = new ArrayList<Expr>();
        exprs.add(expr);
    }

    /** add a new expression to the end of the list */
    public void add(Expr expr)
    {
        exprs.add(expr);
    }
    
    /** Evaluation of list expression will evaluate all of them, until false
     * is encountered - then the evaluation terminates and the result is false.
     * If false is never encountered, the result is the value of the last
     * expression in the list. */
    @Override 
    public Expr eval(Context context) throws Exception
    {
        Expr result = null;
        for(Expr expr: exprs)
        {
            result = expr.eval(context);
            if (result.isFalse()) return new Expr_LOG_CONSTANT(false);
        }
        return result;
    }
}
