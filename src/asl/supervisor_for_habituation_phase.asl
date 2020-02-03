// Agent supervisor in project supervisor

/* Initial beliefs and rules */
/* Initial goals */
!start.

/* Plans */
//TODO check si les srv, action servers sont connectés
//TODO si initServices fail, reprendre la suite du plan une fois qu'ils se sont connectes via retry
+!start : true <-
	configureNode;
	startParameterLoaderNode;
	startROSNode;
	initServices;
	.create_agent(test, "src/asl/habituation_phase.asl", [agentArchClass("arch.RobotAgArch"), beliefBaseClass("rjs.agent.TimeBB"), agentClass("rjs.agent.LimitedAgent")]).

+~connected_srv(S) : true <- .print("service not connected : ", S).
	
	
	