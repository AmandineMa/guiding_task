n(-1).

!start.


+!start : true <- 
	.verbose(0); 
//	display_on_tablet("I know where is the capsule.");
	display_on_tablet("I know what is your next step.");
	human_to_monitor("human_0");
	rjs.jia.log_beliefs.

@iE[atomic] +isAttentive(Human) : not inSession(_,_) & not localising<-
//	display_on_tablet;
	jia.qoi.store_attentive_times(T);
	.concat("human_", Human, HTF);
	human_to_monitor(HTF);
	?n(N);
	-+n(N+1);
	+inSession(Human,N+1)[N+1];
	engage(N+1);
	jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
	if(Dialogue == false){
		text2speech(hello);
//		display_on_tablet(listening);
	}.
	
-inTaskWith(Human,ID) : not  isPerceiving(_, Human)<- 
	?n(N);
	+endTask(Human,ID)[N];
	.wait(6000);
	if(not rjs.jia.believes(isPerceiving(_,Human))){
		!bye(Human);
	}.

-inTaskWith(Human,ID) : isPerceiving(_, Human) <-
	?n(N);
	+endTask(Human,ID)[N].
	
+inTaskWith(Human,ID) : true <- 
	?n(N);
	+startTask(Human,ID)[N].

+!bye(Human) : not inTaskWith(_,_) & inSession(Human,N) & not bye <-
	+bye;
	+overBy(not_perceived)[N];
	jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
	!wait_end_talking;
	if(Dialogue == false){
		//text2speech(goodbye);
	}else{
		terminate_interaction(N);
	}
	!!clean_facts(Human);
//	!loca;
	-bye.
	
+!bye(Human) : inTaskWith(_,_) <- true.

-!bye(Human)[Failure, code(Code),code_line(_),code_src(_),error(_),error_msg(_)] : true <-
	if(.substring(loca, Failure)){
		!loca;
	}.
	
+!wait_end_talking: finished_talking(_) <-
	if(not rjs.jia.believes(finished_talking(true))){
		.wait(300);
		!wait_end_talking;
	}.
	
+!wait_end_talking: true <- true.

+!loca: not localising <-
	+localising;
	jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
	if(Dialogue == false){
		text2speech(localising);
	}
	human_to_monitor("");
	jia.robot.get_param("/guiding/robot_base/position", "List", P);
	jia.robot.get_param("/guiding/robot_base/orientation", "List", O);
	jia.robot.get_param("/guiding/immo", "Boolean", Immo);
	if(Immo == false){
		move_to(map, P, O);	
	}
	localise;
	if(Immo == false){
		move_to(map, P, O);	
	}
	-localising.
	
@loc[max_attempts(2)]+!loca: localising <-
	jia.robot.get_param("/guiding/robot_base/position", "List", P);
	jia.robot.get_param("/guiding/robot_base/orientation", "List", O);
	jia.robot.get_param("/guiding/immo", "Boolean", Immo);
	if(Immo == false){
		move_to(map, P, O);	
	}
	-localising.// !loca.
	
	
-!loca[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)]: true <-
	if(.substring(Error,max_attempts)){
		-localising;
	}else{
		!loca;
	}.
	
+isPerceiving(_, 0) : true <-
	human_to_monitor("human_0").

//-isPerceiving(_, Human) : not inTaskWith(_,_) & inSession(Human,_) & not wait_human <-
//	+wait_human;
//	!wait_human(Human);
//	-wait_human.

+!wait_human(Human) : true <-
	.wait(isPerceiving(_,Human), 6000).
	
-!wait_human(Human) : true <-
	-wait_human;
	!bye(Human).

//received by dialogue
+terminate_interaction(N) : not bye & inSession(Human,N) <- 
	+bye;
	?n(N);
	+overBy(dialogue)[N];
	if(rjs.jia.believes(inTaskWith(Human,ID))){
		?inTaskWith(Human,ID);
		.send(robot, tell, preempted(ID));
	}
	!!clean_facts(Human);
//	!loca;
	-bye.

+!clean_facts(Human): true <-
	?n(N);
	!wait_for_rating(N);
	.findall(B[N,source(X),add_time(Y)],B[N,source(X),add_time(Y)], L);
	rjs.jia.beliefs_to_file(L);
	jia.qoi.qoi_to_file(session, session, N);
	.abolish(_[N]);
	-rating(N,R)[source(_)];
	-isAttentive(Human)[source(_)];
	-terminate_interaction(N)[source(_)].
	
+!wait_for_rating(N): true <-
	.wait(rating(N,R), 10000);
	+rating(N,R)[N].

-!wait_for_rating(Human): true <- true.
	
	
-isAttentive(Human) : inSession(Human,_) <-
	jia.qoi.store_attentive_times.	

+isAttentive(Human): inSession(Human,_) <-
	jia.qoi.store_attentive_times.
	
	
