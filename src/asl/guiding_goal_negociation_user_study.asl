// TODO a voir monitoring piloté par les differents asl
//^!guiding_goal_negociation(ID, Human,_)[state(S)] : S == started | S = resumed <- .resume(monitoring(Human)). 

//^!guiding_goal_negociation(ID, Human, Place)[state(started)] : not started <- +started; +monitoring(ID, Human).	
+!guiding_goal_negociation(ID, Human,Place): true <-
	+guiding_goal_nego(ID, "Timanttiset")[ID].
