package arch.actions.robot;

import java.util.HashMap;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import deictic_gestures.PointAtResponse;
import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;

public class PointAt extends AbstractAction {

	public PointAt(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
	}

	@Override
	public void execute() {
		// to remove the extra ""
		String frame = removeQuotes(actionExec.getActionTerm().getTerm(0).toString());
		boolean with_head = Boolean.parseBoolean(actionExec.getActionTerm().getTerm(1).toString());
		boolean with_base = Boolean.parseBoolean(actionExec.getActionTerm().getTerm(2).toString());

		ServiceResponseListener<PointAtResponse> respListenerP = new ServiceResponseListener<PointAtResponse>() {
			public void onFailure(RemoteException e) {
				handleFailure(actionExec, actionName, e);
			}

			public void onSuccess(PointAtResponse response) {
				actionExec.setResult(response.getSuccess());
				if (!actionExec.getResult())
					actionExec.setFailureReason(new Atom("point_at_failed"),
							"the pointing failed for " + frame);
				rosAgArch.actionExecuted(actionExec);
			}
		};
		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("point", rosnode.buildPointStamped(frame));
		parameters.put("withhead", with_head);
		parameters.put("withbase", with_base);

		rosnode.callAsyncService("point_at", respListenerP, parameters);

	}

}
