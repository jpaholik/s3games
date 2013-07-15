/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package s3games.engine;

import java.util.*;

/**
 *
 * @author petrovic16
 */
public class GameState
{
    /** for each element name, the state number */
    public Map<String,Integer> elementStates; // 1..numStates
    /** for each element name, location name where it currently is placed */
    public Map<String,String> elementLocations;
    /** for each element name, the number of player */
    public Map<String,Integer> elementOwners;
    /** the player number on move 1..N */
    public int currentPlayer;    
    /** the game has finished */
    public boolean gameFinished;
    
    public GameState(GameSpecification specs)
    {
        elementStates = new TreeMap<String, Integer>();
        elementLocations = new TreeMap<String, String>();
        elementOwners = new TreeMap<String, Integer>();
        currentPlayer = 1;
        gameFinished = false;
        
        for (Map.Entry<String, Element> element: specs.elements.entrySet())
        {
            elementStates.put(element.getKey(), element.getValue().initialState);
            elementLocations.put(element.getKey(), element.getValue().initialLocation);
            specs.locations.get(element.getValue().initialLocation).content = element.getValue();
            elementOwners.put(element.getKey(), element.getValue().initialOwner);
        }        
    }
        
    public boolean equals(GameState other)
    {
        if (!elementStates.equals(other.elementStates)) return false;
        if (!elementLocations.equals(other.elementLocations)) return false;
        if (!elementOwners.equals(other.elementOwners)) return false;
        if (currentPlayer != other.currentPlayer) return false;        
        return true;
    }
    
    public GameState getCopy()
    {
       //todo
        return this;
    }
    
    /** compares this state with newState, and returns a move that leads from this state to a new state */
    public Move findMove(GameState newState)
    {
        Move move = null;
        for(Map.Entry<String,String> eLoc: elementLocations.entrySet())
            if (!eLoc.getValue().equals(newState.elementLocations.get(eLoc.getKey())))
            {
                if (move != null) return null;
                else move = new Move(eLoc.getValue(), newState.elementLocations.get(eLoc.getKey()), eLoc.getKey());
            }
        return move;
    }

}
