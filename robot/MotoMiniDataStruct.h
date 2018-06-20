
#ifndef _DATASTRUCT_H_
#define _DATASTRUCT_H_


#define NUM_OF_JOINT 6

/*----------------------------------------*/
/* Data struct to send to MotoMini */
/*----------------------------------------*/
typedef struct{
   double Coord[NUM_OF_JOINT]; //Coordinate of goal
} To_MotoMiniStruct;


/*----------------------------------------------*/
/* Data struct to receive from MotoMini */
/*----------------------------------------------*/
typedef struct{
   double CurAng[NUM_OF_JOINT]; //Current joint angle
} From_MotoMiniStruct;


#endif
