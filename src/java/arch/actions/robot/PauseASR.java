package arch.actions.robot;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractAction;
import rjs.arch.actions.Action;
import rjs.arch.agarch.AbstractROSAgArch;
import std_srvs.EmptyResponse;

public class PauseASR extends AbstractAction implements Action {

	public PauseASR(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}
	
	@Override
	public void execute() {
		ServiceResponseListener<std_srvs.EmptyResponse> respListener = new ServiceResponseListener<std_srvs.EmptyResponse>() {

			@Override
			public void onFailure(RemoteException e) {}

			@Override
			public void onSuccess(EmptyResponse arg0) {}
		};
		actionExec.setResult(true);
		rosnode.callAsyncService("pause_asr", respListener, null);
		rosnode.callAsyncService("web_view_start_processing", respListener, null);
	}

}
