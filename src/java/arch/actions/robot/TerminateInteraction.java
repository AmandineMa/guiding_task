package arch.actions.robot;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import std_srvs.EmptyResponse;

public class TerminateInteraction extends AbstractAction {

	public TerminateInteraction(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
	}

	@Override
	public void execute() {
		ServiceResponseListener<std_srvs.EmptyResponse> respListener = new ServiceResponseListener<std_srvs.EmptyResponse>() {

			@Override
			public void onFailure(RemoteException e) {
				handleFailure(actionExec, actionName, e);
			}

			@Override
			public void onSuccess(EmptyResponse arg0) {
				actionExec.setResult(true);
				rosAgArch.actionExecuted(actionExec);
			}
		};
		rosnode.callAsyncService("terminate_interaction", respListener, null);
	}

}
