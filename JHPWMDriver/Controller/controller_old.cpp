/*
 * The MIT License (MIT)

Copyright (c) 2015 Jetsonhacks

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
// servoExample.cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <JHPWMPCA9685.h>

// UPD Socket
#include <bits/stdc++.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// Split String
#include <string>
#include <iostream>
#include <cstring>


#define PORT     20002
#define MAXLINE 1024

// Calibrated for a Robot Geek RGS-13 Servo
// Make sure these are appropriate for the servo being used!

int servoMin = 120 ;
int servoMax = 719 ;

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

// Map an integer from one coordinate system to another
// This is used to map the servo values to degrees
// e.g. map(90,0,180,servoMin, servoMax)
// Maps 90 degrees to the servo value

int map ( int x, int in_min, int in_max, int out_min, int out_max) {
    int toReturn =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min ;
    // For debugging:
    // printf("MAPPED %d to: %d\n", x, toReturn);
    return toReturn ;
}

int main() {

    // UDP setting
    /////////////////////////////////////////////////////////////////
    int sockfd;
    char buffer[MAXLINE];
    const char *hello = "Received control data";
    struct sockaddr_in servaddr, cliaddr;
       
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
       
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
       
    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);
       
    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, 
            sizeof(servaddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
       
    socklen_t len;
    int n;
   
    len = sizeof(cliaddr);  //len is value/result

    // non-blocking
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);

    /////////////////////////////////////////////////////////////////

    PCA9685 *pca9685 = new PCA9685() ;
    int err = pca9685->openPCA9685();
    if (err < 0){
        printf("Error: %d", pca9685->error);
    } else {
        printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(60) ;
        // 27 is the ESC key
        printf("Hit ESC key to exit\n");
	char* control_data;
	int steer;
	int velocity;
	int brake;
	pca9685->setPWM(1,0,map(80,0,180,servoMin, servoMax));
	sleep(1);
        while(pca9685->error >= 0 && getkey() != 27){

            //pca9685->setPWM(0,0,servoMin) ;
            //pca9685->setPWM(1,0,servoMin) ;

            //sleep(2) ;
            //pca9685->setPWM(0,0,servoMax) ;
	    //pca9685->setPWM(1,0,map(180,0,180,servoMin, servoMax));
            //sleep(1) ;
	    //pca9685->setPWM(1,0,map(0,0,180,servoMin, servoMax));
	    //sleep(1) ;
	    //printf("running\n");

	    n = recvfrom(sockfd, (char *)control_data, MAXLINE, 
                    MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                    &len);
    	    control_data[n] = '\0';
    	    //printf("Client : %s\n", control_data);
    	    sendto(sockfd, (const char *)hello, strlen(hello), 
        	    MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
            	    len);

	    if (n > 1) {
	        steer = atoi(strtok(control_data, "_"));
	        velocity = atoi(strtok(NULL, "_"));
	        brake = atoi(strtok(NULL, "_"));

		pca9685->setPWM(1,0,map(steer,0,180,servoMin, servoMax));
		printf("%i\n", steer);
		sleep(0.1);
	    } else {
	    	printf("No Data\n");
		pca9685->setPWM(1,0,map(80,0,180,servoMin, servoMax));
	    }
        }
        //pca9685->setPWM(1,0,map(0,0,100,servoMin, servoMax));
	//pca9685->setPWM(0,0,map(90,0,180,servoMin, servoMax));
        sleep(1);
    }
    pca9685->closePCA9685();
}
