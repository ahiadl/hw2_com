/* 046267 Computer Architecture - Spring 2016 - HW #2 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <stdbool.h>
//#include <types.h>
/****defines & enums ****/
#define ONES          0xffffffff
#define IP_ROOT       0x0
#define PC_SIZE       32
#define PC_GAP        4
#define PC_BIT_ALIGN  2
#define BIT           1
#define CLEAR_HISTORY 0x0

#define DEBUG 1


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
    
} BtbParams, pBtbParams;

typedef struct {
    bool        valid;
    unsigned    tag;
    unsigned    predictedPc;
    unsigned*   history;
    Prediction* table;
}BtbData, *pBtbData;

typedef struct{
    BtbParams Params;
    pBtbData* Data;
}Btb, *pBtb;

/****Globals****/
Btb GBtb;
SIM_stats stats;


unsigned log_btb (unsigned num){
    unsigned logBtb=0;
    while(num>1){logBtb++; num=num>>1;}
    //if (DEBUG) printf("LOG: %d \n",logBtb );
    return logBtb;
}

/***********************************************************************************************/
int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, bool isGlobalHist, bool isGlobalTable, bool isShare){
    //TODO: think of TAG of size 0
    //TODO: think when init fails
    if(DEBUG) printf("Starting Init\n");
    
    /************Allocating BTB main pointers Array ***********/
    GBtb.Data = (pBtbData*)malloc(sizeof(pBtbData)*btbSize);
    if(GBtb.Data == NULL) return -1; 
    /************Updating local params*************************/
    GBtb.Params.btbSize = btbSize;
    GBtb.Params.historySize = historySize;
    GBtb.Params.isGlobalHist = isGlobalHist;
    GBtb.Params.isGlobalTable = isGlobalTable;  
    GBtb.Params.isShare = isShare;  
    GBtb.Params.tagSize = tagSize;
    
    /******************Allocating globals**********************/
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
    if(DEBUG) printf("done init globals\n");
    
    /*************Allocating & Initializing Locals ************/
    int curBtbData;
    for (curBtbData = 0 ; curBtbData < btbSize; curBtbData ++){
        GBtb.Data[curBtbData] = (pBtbData)malloc(sizeof(BtbData));
        GBtb.Data[curBtbData]->predictedPc = IP_ROOT;
        GBtb.Data[curBtbData]->tag = 0;
        GBtb.Data[curBtbData]->valid = false;

        if(!isGlobalHist){
            GBtb.Data[curBtbData]->history = (unsigned*)malloc(sizeof(unsigned));
            if(GBtb.Data[curBtbData]->history == NULL){
               for (--curBtbData; curBtbData >=0; curBtbData--){
                   free(GBtb.Data[curBtbData]->history);
                   free(GBtb.Data[curBtbData]);
               }
               if(isGlobalTable) free(globaltable);
               free(GBtb.Data);
               if(DEBUG) printf("Failed to allocate History\n");
               return -1;
            }
        } 
        else GBtb.Data[curBtbData]->history = globalhistory;
        *GBtb.Data[curBtbData]->history = 0 ;
        
        if(!isGlobalTable){ 
            GBtb.Data[curBtbData]->table = (Prediction*)malloc(sizeof(Prediction)*(1<<historySize));
            if(GBtb.Data[curBtbData]->table == NULL){
               int histCurBtbData = curBtbData;
               for (--curBtbData; curBtbData >=0; curBtbData--){
                   free (GBtb.Data[curBtbData]->table);
                   if(!isGlobalHist) free(GBtb.Data[curBtbData]->history);
                   free (GBtb.Data[curBtbData]);
               }
               if(isGlobalHist) free(globalhistory);
               else for (--histCurBtbData; histCurBtbData >=0; histCurBtbData--) free (GBtb.Data[histCurBtbData]->history);
               free(GBtb.Data);
               if(DEBUG) printf("Failed to allocate Table\n");
               return -1;
            }
        if(DEBUG) printf("done init iteration: %d\n", curBtbData);
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
    if(DEBUG) printf("done all init\n");
	return 0;
}
/***********************************************************************************************/
bool BP_predict(uint32_t pc, uint32_t *dst){
    if(DEBUG) printf("Started predicting\n");
    int btbIdx = ((pc>>PC_BIT_ALIGN) & (ONES >> (PC_SIZE-log_btb(GBtb.Params.btbSize))));
    if(DEBUG) printf("calculated IDX: %d, btbSize: %d, ONES-btb: 0x%x", btbIdx, GBtb.Params.btbSize, (ONES >> (PC_SIZE-log_btb(GBtb.Params.btbSize))));
    if(DEBUG) printf("pc: 0x%x, pc_size-historysize: %d history: 0x%x\n",pc >> PC_BIT_ALIGN, ONES>>(PC_SIZE-GBtb.Params.historySize), *GBtb.Data[btbIdx]->history);
    int tableCell = (GBtb.Params.isShare && GBtb.Params.isGlobalTable) ? 
                    (((pc >> PC_BIT_ALIGN) & (ONES>>(PC_SIZE-GBtb.Params.historySize)))^(*GBtb.Data[btbIdx]->history)) : *GBtb.Data[btbIdx]->history;
    if(DEBUG) printf("Done calculating IDX and Table Cell 0x%x\n",tableCell);
    if (GBtb.Data[btbIdx]->valid) {
        if(DEBUG) printf("Valid On\n");   
        int tagCalc = (pc>>PC_BIT_ALIGN) & (ONES >> (PC_SIZE-GBtb.Params.tagSize));
        if (tagCalc == GBtb.Data[btbIdx]->tag){
            if(DEBUG) printf("Same Tag\n");
            if(DEBUG) printf("cell content: %d \n",GBtb.Data[btbIdx]->table[tableCell]);
            if (BR_WT == (GBtb.Data[btbIdx]->table[tableCell] & BR_WT)){
                	*dst = GBtb.Data[btbIdx]->predictedPc;
                if(DEBUG) printf("Predicted\n");   
                return true; 
            }
        }
    }

    *dst = pc + PC_GAP;
    return false;
}
/***********************************************************************************************/
void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
//we assume that there are no branch cmds to pc+4 - hence if (targetPc == pred_dst) then the prediction was correct 
   stats.flush_num += ((taken && (pred_dst != targetPc)) || (!taken && (pred_dst != pc+PC_GAP)));
   stats.br_num++;
   int btbIdx = ((pc>>PC_BIT_ALIGN) & (ONES >> (PC_SIZE-log_btb(GBtb.Params.btbSize))));
   int tagCalc = (pc>>PC_BIT_ALIGN) & (ONES >> (PC_SIZE-GBtb.Params.tagSize));   
   int tableCell = (GBtb.Params.isShare && GBtb.Params.isGlobalTable) ? 
                   (((pc >> PC_BIT_ALIGN) & (ONES>>(PC_SIZE-GBtb.Params.historySize)))^(*GBtb.Data[btbIdx]->history)) : *GBtb.Data[btbIdx]->history;
   if (GBtb.Data[btbIdx]->valid && tagCalc == GBtb.Data[btbIdx]->tag) {
       //if (tagCalc == GBtb.Data[btbIdx]->tag){
           *GBtb.Data[btbIdx]->history = (ONES>>(PC_SIZE-GBtb.Params.historySize)) & ((*GBtb.Data[btbIdx]->history << BIT) +taken);
           if(DEBUG) printf("History: %d\n", *GBtb.Data[btbIdx]->history);
           switch (GBtb.Data[btbIdx]->table[tableCell]){
               case BR_ST:  GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_ST  : BR_WT;  break;
               case BR_WT:  GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_ST  : BR_WNT; break;
               case BR_WNT: GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_WT  : BR_SNT; break;
               case BR_SNT: GBtb.Data[btbIdx]->table[tableCell] = taken ? BR_WNT : BR_SNT; break;
               default : break;
           }
           //if(taken) GBtb.Data[btbIdx].predictedPc = targetPc;
      // }
   }else{
       
       if(!GBtb.Params.isGlobalHist)  *GBtb.Data[btbIdx]->history = CLEAR_HISTORY | taken; 
       else *GBtb.Data[btbIdx]->history = (ONES>>(PC_SIZE-GBtb.Params.historySize)) & ((*GBtb.Data[btbIdx]->history << BIT) +taken);
       int cellToDel;
       if(!GBtb.Params.isGlobalTable) for(cellToDel = 0; cellToDel<(BIT<<GBtb.Params.historySize) ; cellToDel++) GBtb.Data[btbIdx]->table[cellToDel] = BR_WNT;  
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
   if(taken) GBtb.Data[btbIdx]->predictedPc = targetPc;
   if(DEBUG) printf("cell content(updated): %d \n",GBtb.Data[btbIdx]->table[tableCell]);
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
