package jia.robot;

import java.util.HashMap;
import java.util.Map;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Term;
import pointing_planner.VisibilityScoreResponse;
import rjs.arch.agarch.AbstractROSAgArch;

public class can_be_visible extends DefaultInternalAction {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
    	String human = args[0].toString().replaceAll("^\"|\"$", "");
    	String place = args[1].toString().replaceAll("^\"|\"$", "");
    	Map<String, Object> parameters = new HashMap<String, Object>();
    	parameters.put("agentname", human);
		parameters.put("targetname", place);
    	VisibilityScoreResponse vis_resp = AbstractROSAgArch.getRosnode().callSyncService("is_visible", parameters);
    	boolean result = false;
		if(vis_resp != null)
			result = vis_resp.getIsVisible();
		return result;
    }
	

}
