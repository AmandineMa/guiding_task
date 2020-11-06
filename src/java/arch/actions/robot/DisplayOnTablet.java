package arch.actions.robot;

import java.util.HashMap;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import jason.asSemantics.ActionExec;
import rjs.arch.actions.AbstractAction;
import rjs.arch.actions.Action;
import rjs.arch.agarch.AbstractROSAgArch;
import std_srvs.EmptyResponse;

public class DisplayOnTablet extends AbstractAction implements Action {

	public DisplayOnTablet(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}
	
	@Override
	public void execute() {
		if(actionExec.getActionTerm().getTerms() == null) {
			ServiceResponseListener<std_srvs.EmptyResponse> respListener = new ServiceResponseListener<std_srvs.EmptyResponse>() {

				@Override
				public void onFailure(RemoteException e) {}

				@Override
				public void onSuccess(EmptyResponse arg0) {}
			};
			rosnode.callAsyncService("reset_tablet", respListener, null);
		}else if(actionExec.getActionTerm().getTerm(0).toString().equals("listening")) {
			ServiceResponseListener<std_srvs.EmptyResponse> respListener = new ServiceResponseListener<std_srvs.EmptyResponse>() {

				@Override
				public void onFailure(RemoteException e) {}

				@Override
				public void onSuccess(EmptyResponse arg0) {}
			};
			rosnode.callAsyncService("display_listening", respListener, null);
		}else {
			ServiceResponseListener<nao_interaction_msgs.StringResponse> respListener = new ServiceResponseListener<nao_interaction_msgs.StringResponse>() {
				@Override
				public void onFailure(RemoteException e) {}
	
				@Override
				public void onSuccess(nao_interaction_msgs.StringResponse response) {}
			};
			Map<String, Object> parameters2 =  new HashMap<String, Object>();
			parameters2.put("request", actionExec.getActionTerm().getTerm(0).toString());
			rosnode.callAsyncService("display_speech", respListener,parameters2);
		}
		actionExec.setResult(true);
	}

}
