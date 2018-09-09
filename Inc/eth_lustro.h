/*
 * eth_lustro.h
 *
 *  Created on: 17.07.2018
 *      Author: Myles
 */

#ifndef ETH_LUSTRO_H_
#define ETH_LUSTRO_H_

#include "stm32f1xx_hal.h"
#include "enc28j60.h"
#include "ip_arp_udp_tcp.h"
#include "EtherShield.h"
#include "comm_lustro.h"

#define TCP_ID_P_H 18
#define TCP_ID_P_L 19
#define REC_BUF_SIZE 200
#define UDP_BUF_SIZE 1500
#define MES_BUF_SIZE 100

#define LUSTRO_DATA_OFFSET 10

extern uint8_t remoteMAC[];
extern uint8_t remoteip[];
extern uint8_t ip[];
extern uint8_t MAC[];
extern uint16_t port;
extern uint16_t port2;
extern uint8_t counter;

extern char message[];
extern uint8_t received[];
extern uint16_t received_size;
extern uint8_t packet[];
extern uint8_t udp_buffer[];

void display_mac();
void send_random_packet();
void display_received_packet(uint16_t size, uint8_t* packet);

uint8_t is_tcp_syn(uint8_t* packet); // check if tcp packet is a syn packet; 0 if is not a syn, 1 is it is
uint8_t is_tcp_synack(uint8_t* packet);
uint8_t is_tcp_ack(uint8_t* packet);
uint8_t is_tcp_fin(uint8_t* packet);
uint8_t is_tcp_psh(uint8_t* packet);
uint8_t is_tcp_rst(uint8_t* packet);
uint8_t is_tcp_pshack(uint8_t* packet); // tcp keep-alive
uint8_t is_tcp_finack(uint8_t* packet); // end transmission request
uint8_t is_tcp_finpshack(uint8_t* packet); // end transmission request
uint8_t tcp_fin_handler(uint8_t* packet); // end transmission handler

uint8_t eth_pktrecv_valid();
uint8_t eth_packet_handler(uint8_t* packet, uint16_t packet_size);
uint16_t eth_pktid(uint8_t* packet);
uint8_t tcp_packet_id_valid(uint16_t packetid);

uint8_t udp_send(uint8_t* packet, uint16_t length);

#endif /* ETH_LUSTRO_H_ */
