// Agent supervisor in project supervisor

/* Initial beliefs and rules */
/* Initial goals */
!start.

/* Plans */
//TODO check si les srv, action servers sont connectés
//TODO si initServices fail, reprendre la suite du plan une fois qu'ils se sont connectes via retry
+!start : true <-
	.verbose(1);
	configureNode;
	startParameterLoaderNode("/guiding.yaml");
	startROSNodeGuiding;
	initServices;
	initGuidingAs; 
//	.create_agent(test, "src/asl/test.asl", [agentArchClass("arch.InteractAgArch"), beliefBaseClass("agent.TimeBB")]).
	.create_agent(interac, "src/asl/interac.asl", [agentArchClass("arch.agarch.InteractAgArch"), beliefBaseClass("rjs.agent.TimeBB"), agentClass("rjs.agent.LimitedAgent")]);
	.create_agent(robot, "src/asl/robot.asl", [agentArchClass("arch.agarch.RobotAgArch"), beliefBaseClass("rjs.agent.TimeBB"), agentClass("rjs.agent.LimitedAgent")]).

+~connected_srv(S) : true <- .print("service not connected : ", S).

-!start [Failure, error(ErrorId), error_msg(Msg), code(CodeBody), code_src(CodeSrc), code_line(CodeLine), source(self)]: true <-
	if(.substring(Failure, "srv_not_connected")){
		!retry_init_services;
	}.

-!start [code(Code),code_line(_),code_src(_),error(_),error_msg(_),source(self)] : true <- true.
	
+!retry_init_services : true <-
	retryInitServices;
	jia.robot.publish_marker(0);
	.print("started");
	.create_agent(interac, "src/asl/interac.asl", [agentArchClass("arch.agarch.InteractAgArch"), beliefBaseClass("rjs.agent.TimeBB"), agentClass("rjs.agent.LimitedAgent")]);
	.create_agent(robot, "src/asl/robot.asl", [agentArchClass("arch.agarch.RobotAgArch"), beliefBaseClass("rjs.agent.TimeBB"), agentClass("rjs.agent.LimitedAgent")]).
	
-!retry_init_services : true <-
	.wait(3000);
	!retry_init_services.
	
+init_over(failed) : true <- .print("robot initialisation failed").

+guiding_goal(ID,Human, Target) : true <- 
	.send(robot, achieve, guiding(ID, Human, Target));
	-guiding_goal(ID, Human, Target);
	+current_guiding_goal(ID).
	
+preempted(ID)[source(A)] : true <-
	if(.substring(A,self)){
		.send(robot, tell, preempted(ID));	
	}
	-current_guiding_goal(ID);
	-cancel_goal(ID).

// To automatically restart a suspended goal after that the other one finished
//-guiding_goal(_,_,_) : suspended_guiding_goal(_) <-
//	if(.count((suspended_guiding_goal(_)),I) & I == 1){
//		?suspended_guiding_goal(ID);
//	}else{
//		rjs.jia.more_recent_bel(suspended_guiding_goal(_), B);
//		B =.. [Sgg, [ID], [_,_]];
//	}
//	.send(robot, tell, resume(ID));
//	-suspended_guiding_goal(ID);
//	+current_guiding_goal(ID).

+end_task(Success, ID) : true <-
	-current_guiding_goal(ID);
	set_guiding_result(Success, ID).
	
	