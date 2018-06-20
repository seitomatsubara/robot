
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
		str.append( my::to_string(coord->elem[i]) );
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

int SocketCommunication::SendAng(To_MotoMiniStruct* ang)
{
	int a1 = 20;
	int a2 = 165;
	int d4 = 165;

	// Param
	double p1 = ang->elem[0];
	double p2 = ang->elem[1];
	double p3 = ang->elem[2];
	double p4 = ang->elem[3];
	double p5 = ang->elem[4];
	double p6 = ang->elem[5];

	// sin & cos
	double
	c1 = cos(p1),	s1 = sin(p1),
	c2 = cos(p2),	s2 = sin(p2),
	c23 = cos(p2+p3),
	s23 = sin(p2+p3),
	c4 = cos(p4),	s4 = sin(p4),
	c5 = cos(p5),	s5 = sin(p5),
	c6 = cos(p6),	s6 = sin(p6);

	// calc T16
	double 	or11, or21, or31,
		or12, or22, or32,
		or13, or23, or33,
		opx, opy, opz;

	or11 =  c23*( c4*c5*c6-s4*s6 ) - s23*s5*c6;
	or21 =  s4*c5*c6+c4*s6;
	or31 = -s23*( c4*c5*c6-s4*s6 ) - c23*s5*c6;
	or12 = -c23*( c4*c5*s6+s4*c6 ) + s23*s5*s6;
	or22 = -s4*c5*s6 + c4*c6;
	or32 =  s23*( c4*c5*s6+s4*c6 ) + c23*s5*s6;
	or13 =  c23*c4*s5 + s23*c5;
	or23 =  s4*s5;
	or33 = -s23*c4*s5 + c23*c5;
	opx  = a1 + a2*s2 + d4*s23;
	opy  = 0;
	opz  = a2*c2 + d4*c23;

	// calc T06
	double r11 = c1*or11 - s1*or21;
	double r21 = s1*or11 + c1*or21;
	double r31 = or31;
	double r12 = c1*or12 - s1*or22;
	double r22 = s1*or12 + c1*or22;
	double r32 = or32;
	double r13 = c1*or13 - s1*or23;
	double r23 = s1*or13 + c1*or23;
	double r33 = or33;
	double px = c1*opx 	+ r13*40;
	double py = s1*opx 	+ r23*40;
	double pz = opz 	+ r33*40;

	// Tmat2euler
	double tx, ty, tz;

	if(abs(r32 -1.0) < 0.001){
   		tx = M_PI/2;
		ty = 0;
		tz = atan2(r21, r11);
	}else if(abs(r32+1.0) < 0.001){
		tx = M_PI/2;
		ty = 0;
		tz = atan2(r21, r11);
	}else{
		tx = asin(r32);
		ty = atan2(-r31,r33);
		tz = atan2(-r13, r22);
	}

	// SendCoord
	To_MotoMiniStruct coord;
	coord.elem[0] = px;
	coord.elem[1] = py;
	coord.elem[2] = pz;
	coord.elem[3] = tx;
	coord.elem[4] = ty;
	coord.elem[5] = tz;

	if ( this->SendCoord(&coord) ) {
		cout << "error: SendAng->SendCoord" << endl;
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

int SocketCommunication::ChangeMode(int mode)
{
	int ret;

	if (mode == 0) {
		ret = write(sock, "mode0", 5);
	} 
	else if (mode == 1) {
		ret = write(sock, "mode1", 5);
	}
	else {
		cout << "mode is wrong." << endl;
	}

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


