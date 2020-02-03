// TODO a voir monitoring piloté par les differents asl
//^!guiding_goal_negociation(ID, Human,_)[state(S)] : S == started | S = resumed <- .resume(monitoring(Human)). 

//^!guiding_goal_negociation(ID, Human, Place)[state(started)] : not started <- +started; +monitoring(ID, Human).	
+!guiding_goal_negociation(ID, Human,Place): true <-
	if(not jia.robot.word_individual(findSub, Place, PlaceOnto)){
		if(jia.robot.word_class(findSub, Place, Class1)){
			if(.list(Class1) & .member("restaurant", Class1)){
				rjs.jia.delete_from_list("restaurant", Class1, ClassL);
				.nth(0, ClassL, Class);
			}else{
				Class=Class1;
			}
			if(jia.robot.word_class(getUp, Class, GU) & .sublist(["product"], GU)){
				.concat(Class, ":sells", Product);
				jia.robot.word_individual(getFrom, Product, List);
			}else{
				.concat(Class, "<1", GetDown);
				if(jia.robot.word_class(getDown, GetDown, TL) & .list(TL)){
					rjs.jia.delete_from_list(Class, TL, List);
				}else{
					jia.robot.word_individual(getType, Class, List);
				}
			}
			
			if(.list(List) & .length(List, X) & X=1){
				.nth(0, List, L);
				jia.robot.word_individual(getType, L, T);
				jia.robot.verba_name(T, ListF);
			}else{			
				ListF=List;
			}
			if(.string(ListF)){
				PlaceOnto=ListF;
//				jia.robot.word_individual(findSub, ListF, PlaceOnto);
			}else{
				jia.robot.verba_name(ListF, PlacesVerba);
				!speak(ID, list_places(PlacesVerba));
				listen(list_places,PlacesVerba);
				?listen_result(list_places,Goal);
				if(not rjs.jia.believes(got_answer(list_places,Goal,_))){
					+got_answer(list_places,Goal,0)[ID];
				}else{
					?got_answer(list_places,Goal,N);
					+got_answer(list_places,Goal,N+1)[ID];
				}
				!guiding_goal_negociation(ID, Human,Goal);
				.succeed_goal(guiding_goal_negociation(ID, Human,Place));
			}
		}else{
			jia.robot.word_individual(findFuzzy, Place, Fuzzy);
			if(.list(Fuzzy)){
				.nth(0,Fuzzy,F);
				jia.robot.word_individual(findSub, F, PlaceOnto);
			}else{
				jia.robot.word_individual(findSub, Fuzzy, PlaceOnto);
			}
			jia.robot.verba_name(PlaceOnto, PlaceVerba);
			!speak(ID, generic(PlaceVerba));
			listen(generic,["yes","no"]);
			?listen_result(generic,Word);
			if(.substring(Word,no)){
				.fail_goal;
			}
		}
	}
	+guiding_goal_nego(ID, PlaceOnto)[ID].
	

// in case of the original plan failure	
-!guiding_goal_negociation(ID, Human, Place)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)]: true <-
	if(.substring(word_individual, Code) | .substring(word_class, Code)){
		+place_not_found(Place)[ID];
		jia.robot.get_param("/guiding/dialogue/hwu", "Boolean", Dialogue);
		if(Dialogue == false){
			!speak(ID, no_place(Place));
		}
		!drop_current_task(ID, guiding_goal_negociation, no_place, Code);
  	}else{
  		!drop_current_task(ID, guiding_goal_negociation, Failure, Code); 
  	}.
	
// in case of the recovery plan failure  	
-!guiding_goal_negociation(ID, Human)[Failure, code(Code),code_line(_),code_src(_),error(Error),error_msg(_)]: true <-
	!drop_current_task(ID, guiding_goal_negoiciation, Failure, Code).
	
