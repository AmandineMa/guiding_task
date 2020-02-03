// Internal action code for project supervisor

package jia.robot;

import java.util.HashMap;
import java.util.Map;

//import java.util.logging.Logger;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Term;
import perspectives_msgs.HasMeshResponse;
import rjs.arch.agarch.AbstractROSAgArch;

public class has_mesh extends DefaultInternalAction {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
    	// to remove the extra ""
		String param = args[0].toString().replaceAll("^\"|\"$", "");

		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("world", "robot/merged");
		parameters.put("name", param);
		HasMeshResponse resp = AbstractROSAgArch.getRosnode().callSyncService("has_mesh", parameters);
		boolean result = false;
		if(resp != null)
			result = resp.getHasMesh();
		return result;
    }

}
