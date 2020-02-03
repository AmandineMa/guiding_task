/******* monitoring **********/	

+monitoring(ID, Human) : true <-
//	.send(interac, tell, monitoring(Human));
	.concat("human_", Human, HTF);
	human_to_monitor(HTF);
	!wait_before_looking(Human).

-monitoring(ID,Human) : true <- 
//	.send(interac, untell, monitoring(Human));
	.succeed_goal(start_monitoring(ID, Human));
	.succeed_goal(wait_before_looking(Human));
	.succeed_goal(look_for_human(Human)).

-isPerceiving(Human) : monitoring(_, Human) <-
	!wait_before_looking(Human).

+!wait_before_looking(Human) : not  look_for_human(Human)<-
	+look_for_human(Human);
	.wait(isPerceiving(Human),6000);
	-look_for_human(Human).
	
+!wait_before_looking(Human) : look_for_human(Human) <- true.
	
-!wait_before_looking(Human) : true <-
	rjs.jia.reset_att_counter(look_for_human);
	!look_for_human(Human).

+!look_for_human(Human) : isPerceiving(Human) <- 
	?task(ID, Task, Human, Place);
	G =.. [Task, [ID,Human,_],[]];
	.resume(G);
	-look_for_human(Human).
	
	
@lfh[max_attempts(2)] +!look_for_human(Human) : not isPerceiving(Human) & monitoring(_, Human) <- 
	?task(ID, Task, Human, Place);
	G =.. [Task, [ID,Human,_],[]];
	jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
	if(Dialogue == false){
		.suspend(G);
		.wait(800);
		!speak(ID,where_are_u);
	}
	.wait(isPerceiving(Human),6000);
	?task(ID, _, Human, _);
	jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
	if(Dialogue == false){
		!speak(ID,found_again);	
		.wait(800);
		.resume(G);
	}
	-look_for_human(Human).
	
+!look_for_human(Human) : isPerceiving(Human) & not monitoring(_, Human) <- -look_for_human(Human).

-!look_for_human(Human)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : isPerceiving(Human) <- 
	?task(ID, Task, Human, _);
	jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
	if(Dialogue == false){
		!speak(ID,found_again);
		G =.. [Task, [ID,Human,_],[]];
		.wait(800);
		.resume(G);
	}
	-look_for_human(Human).
	
-!look_for_human(Human)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)] : not isPerceiving(Human) <- 
	?task(ID, _, Human, _);
	-look_for_human(Human);
	if(.substring(Error,max_attempts)){
		jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
		if(Dialogue == false){
			!speak(ID,cannot_find); 
		}
		.wait(1000);
		.send(interac, tell, left_task(Human));
		!drop_current_task(ID, cannot_find, cannot_find, Code);
	}else{
		!look_for_human(Human);
	}.

//+~isEngagedWith(Human,_)	: need_attentive_human(Human)  & isPerceiving(Human) <-
//	!handle_not_engaged(Human).
//
//+need_attentive_human(Human) : isPerceiving(Human) & not isEngagedWith(Human,_) <-
//	!handle_not_engaged(Human).
//	
//+!handle_not_engaged(Human) : not handling <-
//	+handling;
//	?task(ID, Task, Human, Place);
//	-monitoring(ID, Human);
//	G =.. [Task, [ID,Human,_],[]];
//	.suspend(G);
//	.wait(800);
//	!speak(ID, get_attention);
//	.wait(isEngagedWith(Human,_), 3000);
//	.resume(G);
//	+monitoring(ID, Human);
//	-handling.
//
//+!handle_not_engaged(Human) : handling <- true.
//
//-!handle_not_engaged(Human) : true <-
//	?task(ID, Task, Human, Place);
//	!speak(ID, continue_anyway);
//	G =.. [Task, [ID,Human,_],[]];
//	.resume(G);
//	+monitoring(ID, Human);
//	-handling.
//	
	



