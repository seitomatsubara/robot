#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <cmath>

#include "client.h"

using namespace std;


/*-------------------------*/
/*  SocketCommunication    */
/*-------------------------*/
SocketCommunication::SocketCommunication() : port(11000), ip_address("192.168.100.50")
{
}

SocketCommunication::SocketCommunication(const int port, const char* ip_address) : port(port), ip_address(ip_address)
{
}


int SocketCommunication::Init(void)
{
	struct sockaddr_in server;
	char buf[32];
	int n;

	// make socket
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("socket");
		printf("%d\n", errno);
		return 1;
	}

	server.sin_family = AF_INET;
	server.sin_port = htons(port);
	inet_pton(AF_INET, ip_address, &server.sin_addr.s_addr);

	// connect
	printf("---Connecting to MotoMini---ip:%s, port:%d\n", ip_address,port);
	if ( connect(sock, (struct sockaddr *)&server, sizeof(server)) != 0) {
		perror("connect");
		printf("%d\n", errno);
		return 1;
	}
	printf("Connection Succeeded.\n");

	/*// get ready
	memset(buf, 0, sizeof(buf));
	n = read(sock, buf, sizeof(buf));
	printf("%d, %s\n", n, buf);
	*/
	return 0;
}

namespace my
{
    template < typename T > std::string to_string( const T& v )
    {
        std::ostringstream stm ;
        return ( stm << v ) ? stm.str() : "error" ;
    }
}

int SocketCommunication::SendCoord(To_MotoMiniStruct* coord)
{
	int ret;

	// Struct to string
	std::string str;
	int i;
	for (i=0;i<NUM_OF_JOINT;i++){
		str.append( my::to_string(coord->Coord[i]) );
		if (i < NUM_OF_JOINT-1) str.append(",");
	}
	char cstr[str.size()+1];
	std::strcpy(cstr, str.c_str());
	//printf("%s\n", cstr);

	// Send data
	ret = write(sock, cstr, sizeof(cstr));
	if (ret < 1) {
		perror("write");
		printf("%d\n", errno);
		return 1;
	}
	//printf("%lu %lu %d\n", str.size(),sizeof(cstr),ret);


	// check send
    char buff[128];
    memset(buff, 0, sizeof(buff));

    int bytesRecv = read(sock, buff, sizeof(buff));

    if (bytesRecv < 0) {
        printf("Error: Failed sending data.\n");
		this->Terminate();
        return 1;
	}
	if ( strncmp(buff, "ok", 2) != 0 ) {
      	printf("Error: Failed sending data.\n");
		this->Terminate();
        return 1;
	}		

	return 0;
}

int SocketCommunication::ChangeSpd(double speed)
{
	int ret;

	ret = write(sock, "speed", 5);
	if (ret < 1) {
		perror("speed");
		printf("%d\n", errno);
		return 1;
	}

	// check send
    char buff[128];
    memset(buff, 0, sizeof(buff));

    int bytesRecv = read(sock, buff, sizeof(buff));

    if (bytesRecv < 0) {
        printf("Error: Failed sending data.\n");
		this->Terminate();
        return 1;
	}
	if ( strncmp(buff, "ok", 2) != 0 ) {
      	printf("Error: Failed sending data.\n");
		this->Terminate();
        return 1;
	}		

	// send speed
	std::string str = my::to_string(speed);
	char cstr[str.size()+1];
	std::strcpy(cstr, str.c_str());

	ret = write(sock, cstr, 16);
	if (ret < 1) {
		perror("speed");
		printf("%d\n", errno);
		return 1;
	}

	return 0;
}


int SocketCommunication::ReceiveAng(From_MotoMiniStruct* ang)
{
	int ret = write(sock, "angle", 5);
	if (ret < 1) {
		perror("angle");
		printf("%d\n", errno);
		return 1;
	}

	int bytesRecv;
	int BUFF_MAX = 128;
        char buff[BUFF_MAX];

        memset(buff, 0, sizeof(buff));

        bytesRecv = read(sock, buff, sizeof(buff));

        if (bytesRecv < 0) {
            printf("Error: Failed receiving data.\n");
            return 1;
	}

	//cout << bytesRecv << endl;
	//cout << buff << endl;
	printf("%s\n", buff);
        split( buff, ",", ang);

	printf("Angle: ");
	for (int i=0; i<NUM_OF_JOINT; i++){
		printf("%lf\t", ang->CurAng[i]);
	}
	printf("\n");
	return 0;
}


void SocketCommunication::Terminate(void)
{
	// close
	int ret;
	char cstr[] = "exit";
	ret = write(sock, cstr, sizeof(cstr));
	close(sock);
	printf("Soket Closed\n");
}

int SocketCommunication::split( char *str, const char *delim, From_MotoMiniStruct *ang ) {
    char    *tk;
    int     cnt = 0;

    tk = strtok( str, delim );
    while( tk != NULL && cnt < NUM_OF_JOINT ) {
        ang->CurAng[cnt++] = atof(tk) / 10000;
        tk = strtok( NULL, delim );
    }
    return cnt;
}
/*
void ChangePhi2Ang(To_MotoMiniStruct* coord)
{
	coord->Coord[2] -= M_PI/2;
	coord->Coord[3] *= -1;
	coord->Coord[4] *= -1;
	coord->Coord[5] *= -1;
}
*/

/*----------------*/
/*     Timer      */
/*----------------*/
void Timer::SetStartTime(){
	gettimeofday(&start_time, NULL);
}

void Timer::StartLoopTime(){
	gettimeofday(&tmp_time, NULL);
}

void Timer::SleepLoopTime(double looptime){
	struct timeval now_time;
	gettimeofday(&now_time, NULL);
	double sec = (double)(now_time.tv_sec - tmp_time.tv_sec);
	double micro = (double)(now_time.tv_usec - tmp_time.tv_usec);
	double passed = sec + micro / 1000.0 / 1000.0;	// [ms]

	if (passed < looptime) {
		usleep( (looptime-passed) * 1000.0 * 1000.0 );
	}
}

double Timer::GetElapsedTime(){
	struct timeval now_time;
	gettimeofday(&now_time, NULL);
	double sec = (double)(now_time.tv_sec - start_time.tv_sec);
	double micro = (double)(now_time.tv_usec - start_time.tv_usec);
	double passed = sec + micro / 1000.0 / 1000.0;

	return passed;
}


