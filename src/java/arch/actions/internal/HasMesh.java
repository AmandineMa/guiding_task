package arch.actions.internal;

import java.util.HashMap;
import java.util.Map;

import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceResponseListener;

import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import perspectives_msgs.HasMeshResponse;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;

public class HasMesh extends AbstractAction {

	public HasMesh(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
	}

	@Override
	public void execute() {
		String param = removeQuotes(actionExec.getActionTerm().getTerm(0).toString());

		ServiceResponseListener<HasMeshResponse> respListener = new ServiceResponseListener<HasMeshResponse>() {
			public void onFailure(RemoteException e) {
				handleFailure(actionExec, actionName, e);
			}

			public void onSuccess(HasMeshResponse hasMesh) {
				actionExec.setResult(hasMesh.getHasMesh());
				if (!hasMesh.getHasMesh()) {
					if (hasMesh.getSuccess()) {
						actionExec.setFailureReason(new Atom("has_no_mesh"), param + " does not have a mesh");
					} else {
						actionExec.setFailureReason(new Atom("srv_has_mesh_failed"), "has_mesh service failed");
					}
				}
				rosAgArch.actionExecuted(actionExec);
			}
		};

		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("world", "base");
		parameters.put("name", param);
		rosnode.callAsyncService("has_mesh", respListener, parameters);
	}

}
