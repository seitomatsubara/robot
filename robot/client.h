
#ifndef _CLIENT_H_
#define _CLIENT_H_


#include "MotoMiniDataStruct.h"
#include <sys/time.h>

class SocketCommunication;
class Timer;



class SocketCommunication
{
private:
	int sock;
	const int port;
	const char* ip_address;

	int split( char *str, const char *delim, From_MotoMiniStruct *ang );

public:
	SocketCommunication();
	SocketCommunication(const int port, const char* ip_address);

	int Init(void);					// Start connection
	int SendCoord(To_MotoMiniStruct* coord);	// input coord
	int SendAng(To_MotoMiniStruct* ang);		// input angle [rad]
	int ReceiveAng(From_MotoMiniStruct* ang);	// Recive angle
	int ChangeSpd(double speed);			// [mm/sec] default 50
	int ChangeMode(int mode);			// 0:reach, 1:tmp
	void Terminate(void);				// Close connection

	//void ChangePhi2Ang(To_MotoMiniStruct* torq);	
};


class Timer
{
private:
	struct timeval start_time;
	struct timeval tmp_time;

public:
	void SetStartTime(void);			// StartControl
	void StartLoopTime(void);
	void SleepLoopTime(double looptime);		// looptime[sec]
	double GetElapsedTime(void);
};

#endif
