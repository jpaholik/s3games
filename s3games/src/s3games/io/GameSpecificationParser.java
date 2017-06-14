package s3games.io;

import java.awt.Point;
import java.io.*;
import s3games.engine.*;
import s3games.engine.expr.*;
import s3games.gui.Circular;
import s3games.gui.ImageWithHotSpot;
import s3games.gui.Rectangular;
import s3games.robot.CameraObjectType;
import s3games.robot.RobotLocation;
import s3games.util.*;

/** The parser of the game specification file. The file has the format similar
 * to windows ini-files. Sections are started with [SECTION NAME], and each line
 * is in the variable=value format. An exception is the [EXPRESSIONS] section, 
 * which contains free-text expression definitions */
public class GameSpecificationParser
{
    /** reference to a config that has the folder names */
    private Config config;
    
    /** reference to a logger */
    private GameLogger logger;
    
    /** the constructed game specification object */
    private GameSpecification specs;

    /** list of recognized section names - here as enumeration */
    private enum sections { BOARD, REALBOARD_ELEMENT_TYPES, ELEMENT_TYPES, LOCATION_TYPES, LOCATIONS,
                            PLAYER_NAMES, MOVABLE_ELEMENTS, EXPRESSIONS,
                            SCORINGS, END_OF_GAME, GAME_RULES };

    /** convert a section name to respective enumeration type element */
    private sections getSection(String sectionName) throws Exception
    {
        sectionName=sectionName.toUpperCase();
        if (sectionName.equals("BOARD")) return sections.BOARD;
        if (sectionName.equals("REALBOARD ELEMENT TYPES")) return sections.REALBOARD_ELEMENT_TYPES;
        if (sectionName.equals("ELEMENT TYPES")) return sections.ELEMENT_TYPES;
        if (sectionName.equals("LOCATION TYPES")) return sections.LOCATION_TYPES;
        if (sectionName.equals("LOCATIONS")) return sections.LOCATIONS;
        if (sectionName.equals("PLAYER NAMES")) return sections.PLAYER_NAMES;
        if (sectionName.equals("MOVABLE ELEMENTS")) return sections.MOVABLE_ELEMENTS;
        if (sectionName.equals("EXPRESSIONS")) return sections.EXPRESSIONS;
        if (sectionName.equals("SCORINGS")) return sections.SCORINGS;
        if (sectionName.equals("END OF GAME")) return sections.END_OF_GAME;
        if (sectionName.equals("GAME RULES")) return sections.GAME_RULES;
        logger.error("unrecognized section name '" + sectionName + "'");
        throw new Exception("unrecognized section name");
    }

    /** construct an empty parser */
    public GameSpecificationParser(Config config, GameLogger logger)
    {
        this.config = config;
        this.logger = logger;
    }

    /** are we parsing some named expression? */
    private boolean inExpression;
    
    /** the expression that is being constructed */
    private Expression expression;
    
    /** add one line to the constructed expression */
    private void addExpressionLine(String ln) throws Exception
    {
        if (!inExpression)
        {
            NameWithArgs exprHeader = new NameWithArgs(ln);                   
            expression = new Expression(exprHeader.args);
            specs.expressions.put(exprHeader.baseName, expression);
            inExpression = true;
        }
        else if (ln.toUpperCase().equals("END"))        
            inExpression = false;
        else expression.addLine(ln);
    }

    /** process new entry in the BOARD section */
    private void boardSetting(String var, String val)
    {
        if (var.equals("image")) specs.boardBackgroundFileName = config.imagePath + "/" + val;
    }

    /** the constructed camera object type */
    private CameraObjectType cot;
    /** process new entry in the REAL ELEMENT TYPE section */ 
    private void realElementTypeSetting(String var, String val)
    {
        var = var.toLowerCase();
        if (var.equals("name"))
        {
            cot = new CameraObjectType(val);
            specs.cameraObjectTypes.add(cot);            
        }
        else if (var.equals("state"))
            cot.elState = Integer.parseInt(val);
        else if (var.equals("huemin"))
            cot.hueMin = Double.parseDouble(val);
        else if (var.equals("huemax"))
            cot.hueMax = Double.parseDouble(val);
        else if (var.equals("saturationmin"))
            cot.satMin = Double.parseDouble(val);
        else if (var.equals("saturationmax"))
            cot.satMax = Double.parseDouble(val);
        else if (var.equals("valuemin"))
            cot.valueMin = Double.parseDouble(val);
        else if (var.equals("valuemax"))
            cot.valueMax = Double.parseDouble(val);
        else if (var.equals("sizemin"))
            cot.sizeMin = Integer.parseInt(val);
        else if (var.equals("sizemax"))
            cot.sizeMax = Integer.parseInt(val);
    }
        
    /** the constructed element type */
    private ElementType et;
    /** the currently processed elState of that element */
    private int state;
    /** name of the image for the currently processed elState */
    private String imageName;
    /** process a new ELEMENT TYPE section entry */
    private void elementTypeSetting(String var, String val) throws Exception
    {
        var = var.toLowerCase();
        if (var.equals("name"))
        {
            et = new ElementType(val);
            specs.elementTypes.put(val, et);
        }
        else if (var.equals("states"))
        {
            et.numStates = Integer.parseInt(val);
            et.images = new ImageWithHotSpot[et.numStates];
            state = 0;
        }
        else if (var.equals("image"))
            imageName = val;
        else if (var.equals("point"))
        {
            String[] hotspot = val.split(",");
            try {
                et.images[state] = new ImageWithHotSpot(config.imagePath + "/" + imageName, Integer.parseInt(hotspot[0]), Integer.parseInt(hotspot[1]));               
            } catch (Exception e)
            {                
                logger.error("could not load image from file '" + imageName + "': " + e);
                throw e;
            }
            if (state == 0) 
            {
                ImageWithHotSpot defaultImageForAllStates = et.images[0];            
                for (int st = 1; st < et.numStates; st++)
                    et.images[st] = defaultImageForAllStates;
            }
            state++;
        }
    }

    /** the constructed location type object */
    private LocationType locationType;
    /** process a new entry in the LOCATION TYPE section */
    private void locationTypeSetting(String var, String val)
    {
        var = var.toLowerCase();
        if (var.equals("name"))
            locationType = new LocationType(val);
        if (var.equals("relevant"))
            locationType.relevant = val.toLowerCase().equals("yes");
        else if (var.equals("image"))
            imageName = val;
        else if (var.equals("shape"))
        {
            IndexedName shapeName = new IndexedName(val);
            if (shapeName.baseName.equals("circle"))
                locationType.shape = new Circular(shapeName.index[0]);
            else if (shapeName.baseName.equals("rectangular"))
                locationType.shape = new Rectangular(shapeName.index[0], shapeName.index[1]);
        }
        else if (var.equals("point"))
        {
            String[] hotspot = val.split(",");

            try {
                ImageWithHotSpot img = new ImageWithHotSpot(config.imagePath + "/" + imageName, Integer.parseInt(hotspot[0].trim()), Integer.parseInt(hotspot[1].trim()));
                locationType.image = img;
                specs.locationTypes.put(locationType.name, locationType);
            } catch (Exception e)
            {
                logger.error("could not load image from file '" + imageName + "': " + e);
            }
        }
    }

    /** the constructed location object */
    private Location location;
    /** process a new entry in the LOCATIONS section */
    private void locationSetting(String var, String val) throws Exception
    {
        var = var.toLowerCase();
        if (var.equals("name"))
        {
            location = new Location(val);
            specs.locations.put(val, location);
        }
        else if (var.equals("type"))
        {
            location.type = val;
            LocationType locType = specs.locationTypes.get(val);
            if (locType == null) 
                throw new Exception("Unknown location type " + val);
            location.relevant = locType.relevant;
        }
        else if (var.equals("point"))
        {
            String[] coord = val.split(",");
            location.point = new Point(Integer.parseInt(coord[0].trim()), Integer.parseInt(coord[1].trim()));
        }
        else if (var.equals("camera"))
        {
            String[] coord = val.split(",");
            location.camera = new Point(Integer.parseInt(coord[0].trim()), Integer.parseInt(coord[1].trim()));
        }
        else if (var.equals("robot"))
        {
            try { location.robot = new RobotLocation(val); }
            catch (Exception e)
            {                
                logger.error(e.toString());
                throw e;
            }
        }
    }

    /** process new entry in the PLAYERS section */
    private void playerNameSetting(String var, String val)
    {
        if (var.toLowerCase().equals("score"))        
            specs.initialPlayerScore = Integer.parseInt(val);
        else
        {        
            int n = Integer.parseInt(var) - 1;
            int max = Math.max(n, specs.playerNames.length + 1);
            String[] newNames = new String[max];
            for (int i = 0; i < specs.playerNames.length; i++)
                newNames[i] = specs.playerNames[i];
            newNames[n] = val;
            specs.playerNames = newNames;
        }
    }

    /** the constructed element object */
    private Element el;
    /** process a new entry in the ELEMENTS section */
    private void movableElementSetting(String var, String val) throws Exception
    {
        var = var.toLowerCase();
        if (var.equals("name"))
        {
            el = new Element(val);
            specs.elements.put(val, el);
        }
        else if (var.equals("type"))
            el.type = val;
        else if (var.equals("player"))        
            el.initialOwner = Integer.parseInt(val);
        else if (var.equals("location"))
            el.initialLocation = val;
        else if (var.equals("state"))
            el.initialState = Integer.parseInt(val);
        else if (var.equals("zindex"))
            el.initialZindex = Integer.parseInt(val);
    }

    /** the constructed GAME SCORING object */
    private GameScoring scoring;
    /** process a new entry in the GAME SCORING section */
    private void scoringsSetting(String var, String val) throws Exception
    {
        var = var.toLowerCase();
        if (var.equals("situation"))
        {
            scoring = new GameScoring(Expr.parseExpr(val));
            specs.scorings.add(scoring);
        }
        else if (var.equals("player"))
            scoring.players.add(Expr.parseExpr(val));
        else if (var.equals("score"))
            scoring.amounts.add(Expr.parseExpr(val));
    }

    /** the situation condition for the constructed termination condition object */
    private Expr situation;
    /** process new entry in the END OF GAME section */
    private void endOfGameSetting(String var, String val) throws Exception
    {
        var = var.toLowerCase();
        if (var.equals("situation"))
            situation = Expr.parseExpr(val);
        else if (var.equals("winner"))
            specs.terminationConditions.put(situation, Expr.parseExpr(val));
    }

    /** put a string into quotes */
    private String stringify(String str)
    {
        return "\"" + str + "\"";
    }
    
    /** the constructed game rule */
    private GameRule rule;
    /** process new entry in the GAME RULES section */
    private void rulesSetting(String var, String val) throws Exception
    {
        var = var.toLowerCase();
        if (var.equals("name"))
        {
            rule = new GameRule(val);
            specs.rules.put(val, rule);
        }
        else if (var.equals("element"))
            rule.element = Expr.parseExpr(stringify(val));
        else if (var.equals("state"))
            rule.state = Expr.parseExpr(val);
        else if (var.equals("player"))
            rule.currentPlayer = Expr.parseExpr(val);
        else if (var.equals("from"))
            rule.from = Expr.parseExpr(stringify(val));
        else if (var.equals("to"))
            rule.to = Expr.parseExpr(stringify(val));
        else if(var.equals("tolocations"))
            rule.toLocations = Expr.parseExpr(val);
        else if (var.equals("condition"))
            rule.condition = Expr.parseExpr(val);
        else if (var.equals("awardplayer"))
            rule.scorePlayer.add(Expr.parseExpr(val));
        else if (var.equals("withscore"))
            rule.scoreAmount.add(Expr.parseExpr(val));
        else if (var.equals("followup"))
            rule.action = Expr.parseExpr(val);
    }

    /** store the setting of the specified section */
    private void storeSetting(sections section, String var, String val) throws Exception
    {
        switch (section)
        {
            case BOARD: boardSetting(var, val); break;
            case REALBOARD_ELEMENT_TYPES: realElementTypeSetting(var, val); break;
            case ELEMENT_TYPES: elementTypeSetting(var, val); break;
            case LOCATION_TYPES: locationTypeSetting(var, val); break;
            case LOCATIONS: locationSetting(var, val); break;
            case PLAYER_NAMES: playerNameSetting(var, val); break;
            case MOVABLE_ELEMENTS: movableElementSetting(var, val); break;
            case SCORINGS: scoringsSetting(var, val); break;
            case END_OF_GAME: endOfGameSetting(var, val); break;
            case GAME_RULES: rulesSetting(var, val); break;
        }
    }

    /** verify the rules - we only check that the sizes of array lists scoreAmount and scorePlayer are the same,
     * more thorough verification would be helpful - future work */
    private void checkRules() throws Exception
    {
        for (GameRule r:specs.rules.values())        
            if (r.scoreAmount.size() != r.scorePlayer.size())            
                throw new Exception("'player=N' and 'score=X' in game rule " + r.name + " specification is not paired");        
    }
    
    /** load the game specification for the specified game. The provided argument will be filled with game specification details */
    public boolean load(String gameName, GameSpecification specs) throws Exception
    {
        this.specs = specs;
        specs.playerNames = new String[0];
        String fileName = config.gamesFolder + "/" + gameName;  //gameName; //
        inExpression = false;
        try {
            BufferedReader r = new BufferedReader(new FileReader (fileName));
            sections section = sections.BOARD;

            while (r.ready())
            {
                String ln = r.readLine().trim();
                if (ln.length() < 2) continue;
                if ((ln.charAt(0) == '/') && (ln.charAt(1)=='/')) continue;
                if (ln.charAt(0) == '[')                
                    section = getSection(ln.substring(1, ln.length() - 1));                
                else if (section == sections.EXPRESSIONS)                
                    addExpressionLine(ln);                
                else
                {
                    String[] setting = ln.split("=", 2);
                    if (setting.length != 2)
                        logger.error("urecognized line in game specification: '" + ln + "'");
                    storeSetting(section, setting[0], setting[1]);
                }
            }
            r.close();
            checkRules();
        } catch (Exception e)
        {   
            e.printStackTrace();
            logger.error("could not load game specification from file " + fileName);
            throw e;
        }
        return true;
    }
}
