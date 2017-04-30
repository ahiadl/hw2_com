/* 046267 Computer Architecture - Spring 2016 - HW #2 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <stdlib.h>
/****Data Structures****/

typedef struct {
    unsigned btbSize;
} BtbParams;

typedef struct {
    unsigned tag;
    unsigned predictedPc;
    unsigned* history;
    char* table;
}BtbData, *pBtbData;


/*****Global Variables ***/
BtbParams Params;
pBtbData* BTB;

/****defines & enums ****/
#define ONES 0xffffffff
#define IP_ROOT 0x0
enum {
    BR_SNT = 0,
    BR_WNT = 1,
    BR_WT  = 2,
    BR_ST  = 3,
};


/***********************************************************************************************/
int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, bool isGlobalHist, bool isGlobalTable, bool isShare){
    //TODO: think of TAG of size 0
    //TODO: think when init fails
    /************Allocating BTB main pointers Array ***********/
    BTB = (pBtbData*)malloc(sizeof(pBtbData)*btbSize);
    if(BTB == NULL) return -1; 
    /******************Allocating globals**********************/
    char* globaltable;
    unsigned* globalhistory;

    if(isGlobalHist){
        globalhistory = (unsigned*)malloc(sizeof(unsigned));
        if(globalhistory == NULL) {
            free(BTB);
            return -1;
        }
    }
    if(isGlobalTable){
        globaltable   = (char*)malloc(sizeof(char)*(1<<historySize));
        if(globalhistory == NULL) {
            free(BTB);
            return -1;
        }
    }        
    /*************Allocating & Initializing Locals ************/
    int curBtbData;
    for (curBtbData = 0 ; curBtbData < btbSize; curBtbData ++){
        BTB[curBtbData] = (pBtbData)malloc(sizeof(BtbData));
        BTB[curBtbData]->predictedPc = IP_ROOT;
        BTB[curBtbData]->tag = 0;
        
        if(!isGlobalHist){
            BTB[curBtbData]->history = (unsigned*)malloc(sizeof(unsigned));
            if(BTB[curBtbData]->history == NULL){
               for (--curBtbData; curBtbData >=0; curBtbData--) free (BTB[curBtbData]->history);
               if(isGlobalTable) free(globaltable);
               free(BTB);
               return -1;
            }
        } 
        else BTB[curBtbData]->history = globalhistory;
        *BTB[curBtbData]->history = 0 ;
        
        if(!isGlobalTable){ 
            BTB[curBtbData]->table = (char*)malloc(sizeof(char)*(1<<historySize));
            if(BTB[curBtbData]->table == NULL){
               int histCurBtbData = curBtbData;
               for (--curBtbData; curBtbData >=0; curBtbData--) free (BTB[curBtbData]->table);
               if(isGlobalHist) free(globalhistory);
               else for (--histCurBtbData; histCurBtbData >=0; histCurBtbData--) free (BTB[histCurBtbData]->history);
               free(BTB);
               return -1;
            }
        } 
        else BTB[curBtbData]->table = globaltable; 
        int i;
        for (i=0; i<(1<<historySize); i++) BTB[curBtbData]->table[i] = BR_WNT;
    }
	return 0;
}
/***********************************************************************************************/
bool BP_predict(uint32_t pc, uint32_t *dst){
    int btbIdx = (pc & (ONES >> (32-Params.btbSize)))>>2;
	return false;
}
/***********************************************************************************************/
void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	return;
}
/***********************************************************************************************/
void BP_GetStats(SIM_stats *curStats) {
	return;
}
/***********************************************************************************************/
