package arch.actions.internal;

import java.util.HashMap;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import pointing_planner.VisibilityScoreResponse;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;

public class CanBeVisible extends AbstractAction {

	public CanBeVisible(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
	}

	@Override
	public void execute() {
		String human = removeQuotes(actionExec.getActionTerm().getTerm(0).toString());
		String place = removeQuotes(actionExec.getActionTerm().getTerm(1).toString());

		ServiceResponseListener<VisibilityScoreResponse> respListener = new ServiceResponseListener<VisibilityScoreResponse>() {
			public void onFailure(RemoteException e) {
				handleFailure(actionExec, actionName, e);
			}

			public void onSuccess(VisibilityScoreResponse vis_resp) {
				if (vis_resp.getIsVisible()) {
					actionExec.setResult(true);
					rosAgArch.addBelief("canBeVisibleFor(\"" + place + "\"," + human + ")");
					
				} else {
					rosAgArch.addBelief("~canBeVisibleFor(\"" + place + "\"," + human + ")");
					actionExec.setResult(false);
					actionExec.setFailureReason(new Atom("not_visible"), place + " is not visible");
				}
				rosAgArch.actionExecuted(actionExec);
			}
		};

		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("agentname", human);
		parameters.put("targetname", place);
		rosnode.callAsyncService("is_visible", respListener, parameters);

	}

}
