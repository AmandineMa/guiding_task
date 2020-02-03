package arch.actions.robot;

import java.util.HashMap;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;

public class ReinitLoca extends AbstractAction {

	public ReinitLoca(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
	}

	@Override
	public void execute() {
		ServiceResponseListener<pepper_localisation.responseResponse> respListener = new ServiceResponseListener<pepper_localisation.responseResponse>() {
			public void onFailure(RemoteException e) {
				handleFailure(actionExec, actionName, e);
			}

			public void onSuccess(pepper_localisation.responseResponse loca_resp) {
				actionExec.setResult(loca_resp.getSuccess());
				if (!loca_resp.getSuccess()) {
					actionExec.setFailureReason(new Atom("reinit_loca_failed"), "reinit localisation has failed");
				}
				rosAgArch.actionExecuted(actionExec);
			}
		};
		Map<String, Object> parameters = new HashMap<String, Object>();
		rosnode.callAsyncService("reinit_loca", respListener, parameters);
	}

}
