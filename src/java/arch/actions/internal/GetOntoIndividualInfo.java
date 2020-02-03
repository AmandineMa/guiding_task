package arch.actions.internal;

import java.util.HashMap;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.StringTermImpl;
import ontologenius_msgs.OntologeniusService;
import ontologenius_msgs.OntologeniusServiceResponse;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.utils.Code;

public class GetOntoIndividualInfo extends AbstractAction {

	public GetOntoIndividualInfo(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
	}

	@Override
	public void execute() {
		String param = actionExec.getActionTerm().getTerm(0).toString();
		String individual_o = actionExec.getActionTerm().getTerm(1).toString();
		String individual = removeQuotes(individual_o);
		String belief_name = actionExec.getActionTerm().getTerm(2).toString();

		ServiceResponseListener<OntologeniusServiceResponse> respListener = new ServiceResponseListener<OntologeniusServiceResponse>() {
			public void onFailure(RemoteException e) {
				handleFailure(actionExec, actionName, e);
			}

			public void onSuccess(OntologeniusServiceResponse onto_individual_resp) {
				OntologeniusServiceResponse places = rosnode
						.newServiceResponseFromType(OntologeniusService._TYPE);
				if (onto_individual_resp.getValues().isEmpty()) {
					places.setCode((short) Code.ERROR.getCode());
				} else {
					places.setCode((short) Code.OK.getCode());
					places.setValues(onto_individual_resp.getValues());
				}

				if (places.getCode() == Code.OK.getCode()) {
					ListTermImpl places_list = new ListTermImpl();
					for (String place : places.getValues()) {
						places_list.add(new StringTermImpl(place));
					}
					
					String placeStr = "" + (places_list.size() == 1 ? places_list.get(0) : places_list);
					rosAgArch.addBelief(belief_name + "(" + individual_o+ "," + placeStr + ")");

					actionExec.setResult(true);
					rosAgArch.actionExecuted(actionExec);
				} else {
					actionExec.setResult(false);
					actionExec.setFailureReason(new Atom("individual_not_found"), individual
							+ " has been not been found in the ontology with the param " + param);
					rosAgArch.actionExecuted(actionExec);
				}
			}
		};

		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("action", param);
		parameters.put("param", individual);
		rosnode.callAsyncService("get_individual_info", respListener, parameters);

	}

}
