package arch.actions.internal;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import jason.asSyntax.Term;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import route_verbalization_msgs.VerbalizeRegionRouteResponse;

public class GetRouteVerba extends AbstractAction {

	public GetRouteVerba(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
	}

	@Override
	public void execute() {
		// to remove the extra ""
		@SuppressWarnings("unchecked")
		List<String> route = removeQuotes((List<Term>) actionExec.getActionTerm().getTerm(0));
		String robot_place = removeQuotes(actionExec.getActionTerm().getTerm(1).toString());
		String place = removeQuotes(actionExec.getActionTerm().getTerm(2).toString());

		ServiceResponseListener<VerbalizeRegionRouteResponse> respListener = new ServiceResponseListener<VerbalizeRegionRouteResponse>() {
			public void onFailure(RemoteException e) {
				handleFailure(actionExec, actionName, e);
			}

			public void onSuccess(VerbalizeRegionRouteResponse verba_resp) {
				String verba = new String(verba_resp.getRegionRoute());
				if (verba_resp.getSuccess() & verba != "") {
					actionExec.setResult(true);
					rosAgArch.addBelief("verbalization(\"" + verba + "\")");
				} else {
					actionExec.setResult(false);
					actionExec.setFailureReason(new Atom("route_verba_failed"), "the route verbalization service failed");
				}
				rosAgArch.actionExecuted(actionExec);
			}
		};

		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("route", route);
		parameters.put("startplace", robot_place);
		parameters.put("goalshop", place);
		rosnode.callAsyncService("route_verbalization", respListener, parameters);

	}

}
