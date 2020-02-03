// Internal action code for project supervisor

package jia.robot;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

//import java.util.logging.Logger;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.StringTermImpl;
import jason.asSyntax.Term;
import ontologenius_msgs.OntologeniusService;
import ontologenius_msgs.OntologeniusServiceResponse;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.utils.Code;

public class word_class extends DefaultInternalAction {
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
		OntologeniusServiceResponse onto_class_resp = AbstractROSAgArch.getRosnode().callSyncService("get_class_info", parameters);
		
		boolean result = false;
		if(onto_class_resp != null) {
			OntologeniusServiceResponse places = AbstractROSAgArch.getRosnode().newServiceResponseFromType(OntologeniusService._TYPE);
			if(((OntologeniusServiceResponse) onto_class_resp).getValues().isEmpty()) {
				places.setCode((short) Code.ERROR.getCode());
			}else {
				places.setCode((short) Code.OK.getCode());
				places.setValues(((OntologeniusServiceResponse) onto_class_resp).getValues());
			}
			
			if(places.getCode() == Code.OK.getCode() & !places.getValues().isEmpty()) {
				if(places.getValues().size() == 1)
					result = un.unifies(args[2], new StringTermImpl(places.getValues().get(0)));
				else {
					ListTermImpl l = new ListTermImpl();
					Iterator<String> it = places.getValues().iterator();
					while(it.hasNext()) {
						l.add(StringTermImpl.parseString("\""+it.next()+"\""));
					}
					result = un.unifies(args[2], l);

				}
				
	        }
		}
		return result;
    }
}
