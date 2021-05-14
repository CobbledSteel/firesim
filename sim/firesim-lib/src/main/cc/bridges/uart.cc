//See LICENSE for license details

// Note: struct_guards just as in the headers
#ifdef UARTBRIDGEMODULE_struct_guard

#include "uart.h"
#include <sys/stat.h>
#include <fcntl.h>

#define _XOPEN_SOURCE
#include <stdlib.h>
#include <stdio.h>

#ifndef _WIN32
#include <unistd.h>

// name length limit for ptys
#define SLAVENAMELEN 256

// COSIM-CODE
#include "packet.h"
// COSIM-CODE

/* There is no "backpressure" to the user input for sigs. only one at a time
 * non-zero value represents unconsumed special char input.
 *
 * Reset to zero once consumed.
 */

// This is fine for multiple UARTs because UARTs > uart 0 will use pty, not stdio
char specialchar = 0;

// COSIM-CODE
// Add these functions due to overloading in the uart class
ssize_t net_write(int fd, const void *buf, size_t count) {
	return write(fd, buf, count);	
}

ssize_t net_read(int fd, void *buf, size_t count) {
	return read(fd, buf, count);	
}
// COSIM-CODE

void sighand(int s) {
	switch (s) {
		case SIGINT:
			// ctrl-c
			specialchar = 0x3;
			break;
		default:
			specialchar = 0x0;
	}
}
#endif

uart_t::uart_t(simif_t* sim, UARTBRIDGEMODULE_struct * mmio_addrs, int uartno): bridge_driver_t(sim)
{
	this->mmio_addrs = mmio_addrs;
	this->loggingfd = 0; // unused

	// COSIM-CODE
	// Adapted from: https://www.cs.cmu.edu/afs/cs/academic/class/15213-f99/www/class26/tcpclient.c
	this->hostname = "54.84.238.65";
	this->portno = 10100 + uartno;

	/* socket: create the socket */
	this->sockfd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
	if (this->sockfd < 0) {
		perror("ERROR opening socket");
		exit(0);
	}
	printf("Created socket!\n");

	/* gethostbyname: get the server's DNS entry */
	this->server = gethostbyname(hostname);
	if (this->server == NULL) {
		fprintf(stderr,"ERROR, no such host as %s\n", hostname);
		exit(0);
	}
	printf("Got server's DNS entry!\n");

	/* build the server's Internet address */
	bzero((char *) &this->serveraddr, sizeof(this->serveraddr));
	this->serveraddr.sin_family = AF_INET;
	bcopy((char *)this->server->h_addr, 
			(char *)&this->serveraddr.sin_addr.s_addr, this->server->h_length);
	this->serveraddr.sin_port = htons(this->portno);
	printf("Got server's Internet address!\n");

	/* connect: create a connection with the server */
	while(connect(this->sockfd, (const sockaddr *) &this->serveraddr, sizeof(this->serveraddr)) < 0);
	//  perror("ERROR connecting");
	//  //exit(0);
	//} else {
	//    printf("Succesfully connected to TCP server!\n");
	//}



	// COSIM-CODE

	if (uartno == 0) {
		// signal handler so ctrl-c doesn't kill simulation when UART is attached
		// to stdin/stdout
		struct sigaction sigIntHandler;
		sigIntHandler.sa_handler = sighand;
		sigemptyset(&sigIntHandler.sa_mask);
		sigIntHandler.sa_flags = 0;
		sigaction(SIGINT, &sigIntHandler, NULL);
		printf("UART0 is here (stdin/stdout).\n");
		inputfd = STDIN_FILENO;
		outputfd = STDOUT_FILENO;
	} else {
		// for UARTs that are not UART0, use a PTY
		char slavename[SLAVENAMELEN];
		int ptyfd = posix_openpt(O_RDWR | O_NOCTTY);
		grantpt(ptyfd);
		unlockpt(ptyfd);
		ptsname_r(ptyfd, slavename, SLAVENAMELEN);

		// create symlink for reliable location to find uart pty
		std::string symlinkname = std::string("uartpty") + std::to_string(uartno);
		// unlink in case symlink already exists
		unlink(symlinkname.c_str());
		symlink(slavename, symlinkname.c_str());
		printf("UART%d is on PTY: %s, symlinked at %s\n", uartno, slavename, symlinkname.c_str());
		printf("Attach to this UART with 'sudo screen %s' or 'sudo screen %s'\n", slavename, symlinkname.c_str());
		inputfd = ptyfd;
		outputfd = ptyfd;

		// also, for these we want to log output to file here.
		std::string uartlogname = std::string("uartlog") + std::to_string(uartno);
		printf("UART logfile is being written to %s\n", uartlogname.c_str());
		this->loggingfd = open(uartlogname.c_str(), O_RDWR | O_CREAT, 0644);
	}

// Don't block on reads if there is nothing typed in
fcntl(inputfd, F_SETFL, fcntl(inputfd, F_GETFL) | O_NONBLOCK);
}

uart_t::~uart_t() {
	free(this->mmio_addrs);
	close(this->loggingfd);
}

void uart_t::send() {
	if (data.in.fire()) {
		write(this->mmio_addrs->in_bits, data.in.bits);
		write(this->mmio_addrs->in_valid, data.in.valid);
	}
	if (data.out.fire()) {
		write(this->mmio_addrs->out_ready, data.out.ready);
	}
}

void uart_t::recv() {
	data.in.ready = read(this->mmio_addrs->in_ready);
	data.out.valid = read(this->mmio_addrs->out_valid);
	if (data.out.valid) {
		data.out.bits = read(this->mmio_addrs->out_bits);
	}
}

// COSIM-CODE
void uart_t::process_packet() {
	/* print the server's reply */
	bzero(this->buf, ROBOTICS_COSIM_BUFSIZE);
	//this->n = net_read(this->sockfd, this->buf, ROBOTICS_COSIM_BUFSIZE);
	this->n = net_read(this->sockfd, this->buf, 1);
	if (this->n > 0) {
		//printf("Recieved Packet, length %d. Byte 0: %x\n", this->n, buf[0] & 0xFF);
		switch(buf[0] & 0xFF)
		{
			case CS_GRANT_TOKEN: 
                //printf("Granting token!\n");
				write(this->mmio_addrs->in_ctrl_bits, 1);
				write(this->mmio_addrs->in_ctrl_valid, true);
				break;
            case CS_REQ_CYCLES:
                //printf("Recieved request for cycle budget!\n");
                //printf("Cycle: %d\n", read(this->mmio_addrs->cycle_count));
                bzero(this->buf, ROBOTICS_COSIM_BUFSIZE);
                sprintf(buf, "%d", read(this->mmio_addrs->cycle_budget));
                net_write(this->sockfd, this->buf, strlen(this->buf));
                break;
            case CS_DEFINE_STEP:
                printf("Received request to set cycle step!\n");
                bzero(this->buf, ROBOTICS_COSIM_BUFSIZE);
	            while (net_read(this->sockfd, this->buf, CS_DEFINE_LEN + 1) != CS_DEFINE_LEN + 1); 
                printf("Set cycle step to %d.\n", atoi(buf));
				write(this->mmio_addrs->cycle_step, atoi(buf));
                break;
		}
	}

}
// COSIM-CODE

void uart_t::tick() {
	data.out.ready = true;
	data.in.valid = false;
	do {
		this->recv();
		// COSIM-CODE
		this->process_packet();
		///* send the message line to the server */
		//this->n = net_write(this->sockfd, this->buf, strlen(this->buf));
		//if (this->n < 0) {
		//  perror("ERROR writing to socket");
		//  exit(0);
		//}
		// COSIM-CODE


		if (data.in.ready) {
			char inp;
			int readamt;
			if (specialchar) {
				// send special character (e.g. ctrl-c)
				// for stdin handling
				//
				// PTY should never trigger this
				inp = specialchar;
				specialchar = 0;
				readamt = 1;
			} else {
				// else check if we have input
				readamt = ::read(inputfd, &inp, 1);
			}

			if (readamt > 0) {
				data.in.bits = inp;
				data.in.valid = true;
			}
		}

		if (data.out.fire()) {
			::write(outputfd, &data.out.bits, 1);
			if (loggingfd) {
				::write(loggingfd, &data.out.bits, 1);
			}
		}

		this->send();
		data.in.valid = false;
	} while(data.in.fire() || data.out.fire());
}
//TODO WHEN TO CLOSE?
//close(this->sockfd);

#endif // UARTBRIDGEMODULE_struct_guard
