/*
 * state_Scan_i2c.cpp
 *
 *  Created on: 2015 juillet 13
 *      Author: Fab
 */


#include "Arduino.h"
#include "params.h"
#include <lib_IHM.h>
#include "Wire.h"
#include "state_Scan_i2c.h"
#include "state_Menu_principal.h"

static char success[128];
static char unknow_error[128];

sState* testScan_i2c(){

	static int memPosition = -1;
	static int current_add = 7;

	int Position = myEnc.read();    //position du selecteur modulo le nombre de choix possible

	if(Position != memPosition){  //on affiche que si on change de position
		byte add=0;
		if(Position > memPosition){
			for(int i=current_add+1;i<120;i++){
				if(success[i] || unknow_error[i]){
					add=i;
					break;
				}
			}
		}

		if(Position < memPosition){
			for(int i=current_add-1;i>8;i--){
				if(success[i] || unknow_error[i]){
					add=i;
					break;
				}
			}
		}
		if(add){
			display(add,success, unknow_error);
			current_add = add;
		}
		if((!add && Position < memPosition) || !Position){
			int nb=0;
			for(int i=8;i<120;i++){
				if(success[i])
					nb++;
			}
			afficher(0,1,"%d Devices(s)",nb);
			current_add -= 1;
		}
		memPosition=Position;
	}

	if(!digitalRead(SELECT)){	//si on appui sur select:
		scan_i2c(success, unknow_error);
		int nb=0;
		for(int i=8;i<120;i++){
			if(success[i])
				nb++;
		}
		afficher(0,1,"%d Devices(s)",nb);
		current_add = 7;
	}

	if(!digitalRead(RETOUR)){
		delay(DELAY_BOUNCE);	//anti rebond
		while(!digitalRead(RETOUR));	//attente du relachement du bouton
		return(&sMenu_principal);
	}

    return NULL;
}

void initScan_i2c(sState *prev){
	Wire.begin();
	myEnc.setLimits(0,32767);
	myEnc.write(0);
	myEnc.setMultiplicators(1,5);
	scan_i2c(success, unknow_error);
    }

void deinitScan_i2c(sState *next){
        // Your code here !
    }

sState sScan_i2c={
		0,
        &initScan_i2c,
        &deinitScan_i2c,
        &testScan_i2c
};

void scan_i2c(char success[], char unknow_error[])
{
	afficher(0,1,"Scan ...");
	for(int i=0;i<128;i++){
		success[i] = 0;
		unknow_error[i] = 0;
	}

	for(byte address = 8; address < 120; address++ ){
		Wire.beginTransmission(address);
		byte error = Wire.endTransmission();

		if(error == 0){
			success[address]=1;
		}

		if(error == 4){
			unknow_error[address]=1;
		}
	}
}

void display(byte add, char success[], char unknow_error[]){
	char hexa[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	int low_add = add & 0b00001111;
	int high_add = add >> 4;
	if(success[add] && unknow_error[add]){
		afficher(0,1,"ERROR at 0x%c%c",hexa[high_add], hexa[low_add]);
	}
	else if(success[add]){
		afficher(0,1,"device at 0x%c%c",hexa[high_add], hexa[low_add]);
	}
	else if (unknow_error[add]){
		afficher(0,1,"error at 0x%c%c",hexa[high_add], hexa[low_add]);
	}
	else{
		afficher(0,1,"no device: 0x%c%c",hexa[high_add], hexa[low_add]);
	}

}
