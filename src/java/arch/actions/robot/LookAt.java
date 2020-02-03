package arch.actions.robot;

import java.util.HashMap;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import deictic_gestures.LookAtResponse;
import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;

public class LookAt extends AbstractAction {

	public LookAt(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		// TODO Auto-generated constructor stub
	}

	@Override
	public void execute() {
		String frame = removeQuotes(actionExec.getActionTerm().getTerm(0).toString());
		boolean withBase = Boolean.parseBoolean(actionExec.getActionTerm().getTerm(2).toString());
		ServiceResponseListener<LookAtResponse> respListener = new ServiceResponseListener<LookAtResponse>() {
			public void onFailure(RemoteException e) {
				handleFailure(actionExec, actionName, e);
			}

			public void onSuccess(LookAtResponse lookAtResp) {
				actionExec.setResult(lookAtResp.getSuccess());
				if (!lookAtResp.getSuccess()) {
					actionExec.setFailureReason(new Atom("look_at_failed"), "the look at failed for " + frame);
				}
				rosAgArch.actionExecuted(actionExec);
			}
		};
		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("point", rosnode.build_point_stamped(actionExec, frame));
		parameters.put("withbase", withBase);
		rosnode.callAsyncService("look_at", respListener, parameters);
	}

}
