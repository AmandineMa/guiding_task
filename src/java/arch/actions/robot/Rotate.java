package arch.actions.robot;

import java.util.HashMap;
import java.util.Map;

import jason.asSemantics.ActionExec;
import jason.asSyntax.Atom;
import jason.asSyntax.ListTerm;
import pepper_resources_synchronizer_msgs.MetaStateMachineRegisterResponse;
import rjs.arch.actions.AbstractAction;
import rjs.arch.agarch.AbstractROSAgArch;
import rjs.utils.Quaternion;
import ros.RosNodeGuiding;

public class Rotate extends AbstractAction {
	
	public Rotate(ActionExec actionExec, AbstractROSAgArch rosAgArch) {
		super(actionExec, rosAgArch);
		setSync(true);
	}

	@Override
	public void execute() {
		ListTerm quaternion = (ListTerm) actionExec.getActionTerm().getTerm(0);

		Quaternion q = Quaternion.create(quaternion);
		double d = q.getYaw();

		Map<String, Object> parameters = new HashMap<String, Object>();
		parameters.put("statemachinepepperbasemanager", ((RosNodeGuiding) rosnode).buildStateMachinePepperBaseManager(actionName, (float) d));
		parameters.put("header", ((RosNodeGuiding) rosnode).buildMetaHeader());

		MetaStateMachineRegisterResponse response = rosnode.callSyncService("pepper_synchro", parameters);

		actionExec.setResult(response != null);
		if(response == null) {
			actionExec.setFailureReason(new Atom("cannot_rotate"), "rotation failed");
		}
	}
}
