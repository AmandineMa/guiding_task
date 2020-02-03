// Internal action code for project supervisor

package jia.robot;

import java.util.HashMap;
import java.util.Map;

//import java.util.logging.Logger;

import jason.asSemantics.*;
import jason.asSyntax.*;
import ontologenius_msgs.OntologeniusService;
import ontologenius_msgs.OntologeniusServiceResponse;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.utils.Code;

public class word_individual extends DefaultInternalAction {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
		String action = args[0].toString();
		action = action.replaceAll("^\"|\"$", "");
    	String param = args[1].toString();
		param = param.replaceAll("^\"|\"$", "");
		
		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("action", action);
		parameters.put("param", param);
		OntologeniusServiceResponse onto_individual_resp = AbstractROSAgArch.getRosnode().callSyncService("get_individual_info", parameters);
		
		boolean result = false;
		if(onto_individual_resp != null) {
			OntologeniusServiceResponse places = AbstractROSAgArch.getRosnode().newServiceResponseFromType(OntologeniusService._TYPE);
			if(onto_individual_resp.getValues().isEmpty()) {
				places.setCode((short) Code.ERROR.getCode());
			}else {
				places.setCode((short) Code.OK.getCode());
				places.setValues(onto_individual_resp.getValues());
			}
			
			if(places.getCode() == Code.OK.getCode() & !places.getValues().isEmpty()) {
				if(places.getValues().size() == 1) {
					result = un.unifies(args[2], new StringTermImpl(places.getValues().get(0)));
				}else {
					ListTermImpl places_list = new ListTermImpl();
					for(String place : places.getValues()) {
						places_list.add(new StringTermImpl(place));
					}
					result = un.unifies(args[2], places_list);
				}
	        }
		}
		return result;
    }
}
