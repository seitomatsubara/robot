
#ifndef _DATASTRUCT_H_
#define _DATASTRUCT_H_


#define NUM_OF_JOINT 6

/*----------------------------------------*/
/* Data struct to send to MotoMini */
/*----------------------------------------*/
typedef struct{
   double elem[NUM_OF_JOINT]; //element of sending data
} To_MotoMiniStruct;


/*----------------------------------------------*/
/* Data struct to receive from MotoMini */
/*----------------------------------------------*/
typedef struct{
   double CurAng[NUM_OF_JOINT]; //Current joint angle
} From_MotoMiniStruct;


#endif
