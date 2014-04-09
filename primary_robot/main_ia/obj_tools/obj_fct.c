/*
 * obj_fct.c
 *
 *  Created on: 29 mars 2014
 *      Author: seb
 */

#include "obj_fct.h"


sTrajEl_t tabEl[4]={ //Trajectory for tree upwards vertical in the origin of the table for tree
	{{0.  ,  0.},{15.2,-16.9},{{33.2 ,-16.9}, 18. , 0., 1.}, 0. , 0., 0.},
	{{17.3,-8.5},{17.3, -8.5},{{-4.9 ,  3.4},-25.2, 0., 1.}, 0. , 0., 1.},
	{{17.3,15.2},{17.3, 15.2},{{33.3 , 23.7}, 18.1, 0., 1.}, 0. , 0., 2.},
	{{15.2,23.7},{15.2, 23.7},{{0.   , 0.  }, 0.  , 0., 1.}, 0. , 0., 3.}
	};

sTrajEl_t tabSeg[2]={
	{{0.  ,  0.},{-10. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 0.},
	{{-10  ,  0.},{-10. , 0.},{{0. ,0.}, 0. , 0., 1.}, 0. , 0., 1.}
	};

void obj_tree(iABObs_t obj)
    {
	static int state=0; //for separate the init, loop and end
    int i;
    sTrajEl_t tabTemp[4];

    switch(state){
	    case 0:
	        printf("Debut objectif arbre\n");
	        for(i=0; i<4 ; i++){  //disable entry point
				obs[N-i-2].active=0;
				obs_updated[N-i-2]++;
	        	}

	        //TODO Temporaire a afiner en fonction si l'objectif est bientot terminer
	        listObj[obj].active=0;
	        bac.nb_point=bac.nb_point+3; //TODO A actualiser en fonction des fruits recoltés


	        memcpy(&tabTemp[0],&tabEl[0], sizeof(tabEl[0])*4);

	        if((pt_select.x==listObj[obj].entryPoint[0].c.x) && (pt_select.y==listObj[obj].entryPoint[0].c.y)){
	        	for(i=0; i<4 ; i++){
	        		switch(obj){
	        			case 0:
							SymElTraj(&tabTemp[i], 1, 0);
							TransElTraj(&tabTemp[i], ((Obj_arbre*)listObj[obj].typeStruct)->x, ((Obj_arbre*)listObj[obj].typeStruct)->y);
							break;
	        			case 1 :
	        			case 2 :
							Rot90Traj(&tabTemp[i]);
							SymElTraj(&tabTemp[i], 0, 1);
							TransElTraj(&tabTemp[i], ((Obj_arbre*)listObj[obj].typeStruct)->x, ((Obj_arbre*)listObj[obj].typeStruct)->y);
							break;
	        			case 3 :
	        				SymElTraj(&tabTemp[i], 0, 1);
	        				TransElTraj(&tabTemp[i], ((Obj_arbre*)listObj[obj].typeStruct)->x, ((Obj_arbre*)listObj[obj].typeStruct)->y);
	        				break;
	        			}
	        		}
	            printf("Sortie 2 selectionné\n");
	        	}
	        else{
	 		   for(i=0; i<4 ; i++){
					switch(obj){
						case 0:
							TransElTraj(&tabTemp[i], ((Obj_arbre*)listObj[obj].typeStruct)->x, ((Obj_arbre*)listObj[obj].typeStruct)->y);
							break;
						case 1 :
						case 2 :
							Rot90Traj(&tabTemp[i]);
							TransElTraj(&tabTemp[i], ((Obj_arbre*)listObj[obj].typeStruct)->x, ((Obj_arbre*)listObj[obj].typeStruct)->y);
							break;
						case 3 :
							SymElTraj(&tabTemp[i], 1, 1);
							TransElTraj(&tabTemp[i], ((Obj_arbre*)listObj[obj].typeStruct)->x, ((Obj_arbre*)listObj[obj].typeStruct)->y);
							break;
						}
					}
				   pt_select=(listObj[obj].entryPoint[0]).c;
				   printf("Sortie 1 selectionné\n");
	        	}
            pt_select=(tabTemp[3]).p1;
            tabTemp[0].p1=obs[0].c;

            //send path
            path.path=&tabTemp[0];
            path.path_len=4;
            send_robot(path) ;

            obs[N-1].c=pt_select;
            obs_updated[N-1]++;

            obs[obj+1].r=R_ROBOT + 5;
            obs_updated[obj+1]++;

            //generique fonction
    		last_time=millis();
    		mode_obj=1;
    		state=1;

            printf("start x=%f y=%f end x=%f y=%f\n",obs[0].c.x,obs[0].c.y,obs[N-1].c.x,obs[N-1].c.y);
	        break;
	    case 1:
			if ((fabs(pt_select.x-_current_pos.x)<1 && fabs(pt_select.y-_current_pos.y)<1)){
				state=2;
				}
	    	break;
	    case 2 :
		    mode_obj=0;
		    state=0;
		    pt_select.x=0;
		    pt_select.y=0;
		    listObj[obj].dist=0;
		   break;
	    default :
	    	printf("Error in obj_tree : state != (0 || 1 || 2\n");
			break;
    	}
    }


void obj_fire(iABObs_t  obj){
	static int state=0, theta=0; //for separate the init, loop and end
    int i;
    sTrajEl_t tabTemp[4];

    switch(state){
	    case 0:
	        printf("Debut objectif feux\n\n");
	        for(i=0; i<4 ; i++){  //disable entry point
				obs[N-i-2].active=0;
				obs_updated[N-i-2]++;
	        	}
	        obs[listObj[obj].numObs[0]].active=0;
	        obs_updated[listObj[obj].numObs[0]]++;

	        listObj[obj].active=0;

	        if(((Obj_feu*)listObj[obj].typeStruct)->pos==3){
				theta=listObj[obj].entryPoint[0].angleEP;
		        memcpy(&tabTemp[0],&tabSeg[0], sizeof(tabSeg[0])*2);
		        pt_select=listObj[obj].entryPoint[0].c;

				//Rotation du segment

				tabTemp[0].p2.x= -10*cos(theta*M_PI/180);
				tabTemp[0].p2.y= -10*sin(theta*M_PI/180);

				tabTemp[1].p1=tabTemp[0].p2;
				tabTemp[1].p2=tabTemp[0].p2;

				//Translation du segment
				TransElTraj(&tabTemp[0], listObj[obj].entryPoint[0].c.x, listObj[obj].entryPoint[0].c.y);
				tabTemp[1].p1=tabTemp[0].p2;
				tabTemp[1].p2=tabTemp[0].p2;

				//Update extremum
	            pt_select=(tabTemp[1]).p1;
	            tabTemp[0].p1=obs[0].c;

	            //send path
	            printf("p1.x=%f, p1.y=%f, p2.x=%f, p2.y=%f \net  p1.x=%f, p1.y=%f, p2.x=%f, p2.y=%f\n",tabTemp[0].p1.x,tabTemp[0].p1.y,tabTemp[0].p2.x,tabTemp[0].p2.y,tabTemp[1].p1.x,tabTemp[1].p1.y,tabTemp[1].p2.x,tabTemp[1].p2.y);
	            path.path=&tabTemp[0];
	            path.path_len=2;
	            send_robot(path) ;

	            obs[N-1].c=pt_select;
	            obs_updated[N-1]++;

	            obs[obj+1].r=R_ROBOT + 5;
	            obs_updated[obj+1]++;

				}

            //generique fonction
    		last_time=millis();
    		mode_obj=1;
    		state=1;
    		//printf("Stop getchar\n");
    		//getchar();
            printf("start x=%f y=%f end x=%f y=%f\n",obs[0].c.x,obs[0].c.y,obs[N-1].c.x,obs[N-1].c.y);
	        break;
	    case 1:
			if ((fabs(pt_select.x-_current_pos.x)<1 && fabs(pt_select.y-_current_pos.y)<1)){
				state=2;
				}
	    	break;
	    case 2 :
	        if(((Obj_feu*)listObj[obj].typeStruct)->pos==3){
				((Obj_feu*)listObj[obj].typeStruct)->c.x += -10*cos(theta*M_PI/180);
				((Obj_feu*)listObj[obj].typeStruct)->c.y += -10*sin(theta*M_PI/180);
				obs[listObj[obj].numObs[0]].c.x=((Obj_feu*)listObj[obj].typeStruct)->c.x ;
				obs[listObj[obj].numObs[0]].c.y=((Obj_feu*)listObj[obj].typeStruct)->c.y ;
		        obs[listObj[obj].numObs[0]].active=1;
		        //printf("Rayon=%f",obs[listObj[obj].numObs[0]].r);
		        //getchar();
				obs_updated[listObj[obj].numObs[0]]++;
				}

		    mode_obj=0;
		    state=0;
		    pt_select.x=0;
		    pt_select.y=0;
		    listObj[obj].dist=0;
		   break;
	    default :
	    	printf("Error in obj_tree : state != (0 || 1 || 2\n");
			break;
    	}
	}

void obj_bac(iABObs_t obj)
    {
    listObj[obj].active=0;
  //  obs[listObj[obj].numObs[0]].active=0;
    obs_updated[listObj[obj].numObs[0]]++;
    bac.nb_point=0;
    /*
    static int state=0; //for separate the init, loop and end
    int i;

    switch(state){
	    case 0:
            //generique fonction
    		last_time=millis();
    		mode_obj=1;
    		state=1;

            printf("start x=%f y=%f end x=%f y=%f\n",obs[0].c.x,obs[0].c.y,obs[N-1].c.x,obs[N-1].c.y);
	        break;
	    case 1:
	    	if( (millis()-last_time)>1000){
	            last_time=millis();
	            if ((fabs(pt_select.x-_current_pos.x)<1 && fabs(pt_select.y-_current_pos.y)<1)){
	            	state=2;
	            	}
	    		}
	    	break;
	    case 2 :
		    mode_obj=0;
		    state=0;
		    pt_select.x=0;
		    pt_select.y=0;
		    listObj[obj].dist=0;
		   break;
	    default :
	    	printf("Error in obj_fire : state != (0 || 1 || 2\n");
			break;
    	}
    */
    #if DEBUG
        printf("Objectif : bac fini\n\n\n");
    #endif
    }
