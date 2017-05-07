/* 046267 Computer Architecture - Spring 2016 - HW #2 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/****defines & enums ****/
#define ONES          0xffffffff
#define IP_ROOT       0x0
#define PC_SIZE       32
#define PC_GAP        4
#define PC_BIT_ALIGN  2
#define BIT           1
#define CLEAR_HISTORY 0x0

typedef enum {
    BR_SNT = 0,
    BR_WNT = 1,
    BR_WT  = 2,
    BR_ST  = 3,
}Prediction;

/****Data Structures****/

typedef struct {
    unsigned btbSize;
    unsigned tagSize;
    unsigned historySize;
    bool isGlobalHist;
    bool isGlobalTable;
    bool isShare;
    
} BtbParams;

typedef struct {
    bool        valid;				//valid is true once the BTB line is updated for the first time
    unsigned    tag;
    unsigned    predictedPc;		//Will contain the branch destination address
    unsigned*   history;			//if global, points to global BHR. if local, will assign a BHR in BP_init
    Prediction* table;				//if global, points to global table. if local, will assign a table in BP_init
}BtbData, *pBtbData;

typedef struct{
    BtbParams Params;
    pBtbData* Data;
}Btb, *pBtb;

/****Globals****/
Btb GBtb;
SIM_stats stats;

unsigned log_btb (unsigned num){	//the log function is needed only for values 2^n, 0<n<5
    unsigned logBtb=0;
    while(num>1){logBtb++; num=num>>1;}
    return logBtb;
}

/***********************************************************************************************/
int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, bool isGlobalHist, bool isGlobalTable, bool isShare){
    /************Allocating BTB main pointers Array ***********/
    GBtb.Data = (pBtbData*)malloc(sizeof(pBtbData)*btbSize);
    if(GBtb.Data == NULL) return -1; 
    /************Updating local parameters*********************/
    GBtb.Params.btbSize = btbSize;
    GBtb.Params.historySize = historySize;
    GBtb.Params.isGlobalHist = isGlobalHist;
    GBtb.Params.isGlobalTable = isGlobalTable;  
    GBtb.Params.isShare = isShare;  
    GBtb.Params.tagSize = tagSize;
    
    /******************Allocating globals if needed*************/
    unsigned* globaltable;
    unsigned* globalhistory;

    if(isGlobalHist){
        globalhistory = (unsigned*)malloc(sizeof(unsigned));
        if(globalhistory == NULL) {
            free(GBtb.Data);
            return -1;
        }
    }
    if(isGlobalTable){
        globaltable   = (unsigned*)malloc(sizeof(unsigned)*(1<<historySize));
        if(globalhistory == NULL) {
            free(GBtb.Data);
            return -1;
        }
    }
    /*************Allocating & Initializing Locals ************/
    int curBtbData;
    for (curBtbData = 0 ; curBtbData < btbSize; curBtbData ++){							//each iteration will init one row in the BTB
        GBtb.Data[curBtbData] = (pBtbData)malloc(sizeof(BtbData));
        GBtb.Data[curBtbData]->predictedPc = IP_ROOT;									//arbitrary choice of IP
        GBtb.Data[curBtbData]->tag = 0;													//arbitrary choice of tag
        GBtb.Data[curBtbData]->valid = false;											//indicates uninitialized row in the BTB

        if(!isGlobalHist){
            GBtb.Data[curBtbData]->history = (unsigned*)malloc(sizeof(unsigned));
            if(GBtb.Data[curBtbData]->history == NULL) return -1;						//disregarded the need to free
        } 
        else GBtb.Data[curBtbData]->history = globalhistory;
        *GBtb.Data[curBtbData]->history = 0 ;
        
        if(!isGlobalTable){ 
            GBtb.Data[curBtbData]->table = (Prediction*)malloc(sizeof(Prediction)*(1<<historySize));
            if(GBtb.Data[curBtbData]->table == NULL) return -1;				   		    //disregarded the need to free
        } 
        else GBtb.Data[curBtbData]->table = globaltable; 
        int i; for (i=0; i<(1<<historySize); i++) GBtb.Data[curBtbData]->table[i] = BR_WNT;
    }
    stats.br_num = 0;
    stats.flush_num = 0;
    stats.size =  ( isGlobalHist  &&  isGlobalTable) ? btbSize*(tagSize + PC_SIZE - PC_BIT_ALIGN) + historySize  + 2*(1<<(historySize))          :
                  ( isGlobalHist  && !isGlobalTable) ? btbSize*(tagSize + PC_SIZE - PC_BIT_ALIGN) + historySize  + 2*(1<<(historySize))*btbSize  :
                  (!isGlobalHist  &&  isGlobalTable) ? btbSize*(tagSize + PC_SIZE - PC_BIT_ALIGN  + historySize) + 2*(1<<(historySize))          :
                  (!isGlobalHist  && !isGlobalTable) ? btbSize*(tagSize + PC_SIZE - PC_BIT_ALIGN  + historySize  + 2*(1<<(historySize)))         : 0;
	//size is calculated according to the formula in the tutorial
    return 0;
}
/***********************************************************************************************/
bool BP_predict(uint32_t pc, uint32_t *dst){
    int btbIdx = ((pc>>PC_BIT_ALIGN) & (ONES >> (PC_SIZE-log_btb(GBtb.Params.btbSize))));		//index of the BTB row
    if (GBtb.Params.btbSize == 1) btbIdx = 0;													//>> operator doesn't work in this case
    int tableCell = (GBtb.Params.isShare && GBtb.Params.isGlobalTable) ?						//if isShare==1, using XOR here
                    (((pc >> PC_BIT_ALIGN) & (ONES>>(PC_SIZE-GBtb.Params.historySize)))^(*GBtb.Data[btbIdx]->history)) : *GBtb.Data[btbIdx]->history;
    if (GBtb.Data[btbIdx]->valid) {																//means there's an entry in this row
        int tagCalc = (PC_SIZE-GBtb.Params.tagSize == 0) ? 0 : (pc>>PC_BIT_ALIGN) & (ONES >> (PC_SIZE-GBtb.Params.tagSize));
        if (tagCalc == GBtb.Data[btbIdx]->tag){													//if tags are same, use the prediction
            if (BR_WT == (GBtb.Data[btbIdx]->table[tableCell] & BR_WT)){						//if MSB is '1', prediction is T
                	*dst = GBtb.Data[btbIdx]->predictedPc;
                return true; 
            }
        }
    }
    *dst = pc + PC_GAP;																			//else, pc+4 is the prediction
    return false;
}
/***********************************************************************************************/
void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
   stats.flush_num += ((taken && (pred_dst != targetPc)) || (!taken && (pred_dst != pc+PC_GAP)));
   stats.br_num++;
   int btbIdx = ((pc>>PC_BIT_ALIGN) & (ONES >> (PC_SIZE-log_btb(GBtb.Params.btbSize))));
   if (GBtb.Params.btbSize == 1 )btbIdx = 0;
   int tagCalc = (PC_SIZE-GBtb.Params.tagSize == 0) ? 0 : (pc>>PC_BIT_ALIGN) & (ONES >> (PC_SIZE-GBtb.Params.tagSize));
   int tableCell = (GBtb.Params.isShare && GBtb.Params.isGlobalTable) ? 
                   (((pc >> PC_BIT_ALIGN) & (ONES>>(PC_SIZE-GBtb.Params.historySize)))^(*GBtb.Data[btbIdx]->history)) : *GBtb.Data[btbIdx]->history;
   if (GBtb.Data[btbIdx]->valid && tagCalc == GBtb.Data[btbIdx]->tag) {							//if valid and tags identical, update
           *GBtb.Data[btbIdx]->history = (ONES>>(PC_SIZE-GBtb.Params.historySize)) & ((*GBtb.Data[btbIdx]->history << BIT) +taken);
           switch (GBtb.Data[btbIdx]->table[tableCell]){
               case BR_ST:  GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_ST  : BR_WT;  break;
               case BR_WT:  GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_ST  : BR_WNT; break;
               case BR_WNT: GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_WT  : BR_SNT; break;
               case BR_SNT: GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_WNT : BR_SNT; break;
               default : break;
           }
   }else{																						//else, initialize entry
       
       if(!GBtb.Params.isGlobalHist){															//initializing history only if local
    	   *GBtb.Data[btbIdx]->history = CLEAR_HISTORY | taken;
    	   tableCell = (GBtb.Params.isShare && GBtb.Params.isGlobalTable) ?						//choosing which FSM to update
    	                      (((pc >> PC_BIT_ALIGN) & (ONES>>(PC_SIZE-GBtb.Params.historySize)))^ 0) : 0;
       }
       else *GBtb.Data[btbIdx]->history = 														//updating global history
    		   (ONES>>(PC_SIZE-GBtb.Params.historySize)) & ((*GBtb.Data[btbIdx]->history << BIT) +taken);
       int cellToDel;
       if(!GBtb.Params.isGlobalTable)															//initializing local table
    	   for(cellToDel = 0; cellToDel<(BIT<<GBtb.Params.historySize) ; cellToDel++) GBtb.Data[btbIdx]->table[cellToDel] = BR_WNT;
       switch (GBtb.Data[btbIdx]->table[tableCell]){
           case BR_ST:  GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_ST  : BR_WT;  break;
           case BR_WT:  GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_ST  : BR_WNT; break;
           case BR_WNT: GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_WT  : BR_SNT; break;
           case BR_SNT: GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_WNT : BR_SNT; break;
           default : break;
       }
       GBtb.Data[btbIdx]->tag = tagCalc;
       GBtb.Data[btbIdx]->valid = true;
   }  
   GBtb.Data[btbIdx]->predictedPc = targetPc;													//updating the destination in the BTB
   return;
}
/***********************************************************************************************/
void BP_GetStats(SIM_stats *curStats) {
    curStats->flush_num = stats.flush_num;
    curStats->br_num = stats.br_num;
    curStats->size = stats.size;
	return;
}
/***********************************************************************************************/
