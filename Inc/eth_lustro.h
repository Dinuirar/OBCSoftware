/*! \file
 * \brief Ethernet control
 *
 * Functions and variables for the ethernet control
 * */

#ifndef ETH_LUSTRO_H_
#define ETH_LUSTRO_H_

#include "stm32f1xx_hal.h"
#include "enc28j60.h"
#include "ip_arp_udp_tcp.h"
#include "EtherShield.h"
#include "comm_lustro.h"

/**
 *  @defgroup ETHERNET Ethernet
 * \brief Ethernet communication with the ground station
 *  @{
 */
/// \brief Position of higher ID-byte in TCP packet
#define TCP_ID_P_H 18
/// \brief Position of lower ID-byte in TCP packet
#define TCP_ID_P_L 19
/// \brief Receive buffer size
#define REC_BUF_SIZE 200
/// \brief UDP buffer size
#define UDP_BUF_SIZE 1500
/// \brief UART messages buffer size
#define MES_BUF_SIZE 100


/// \brief Offset for walkaround due to hardware-packet-padding in ENC28J60
#define LUSTRO_DATA_OFFSET 10

/// \brief MAC address of the Ground Station
extern uint8_t remoteMAC[];

/// \brief IP address of the Ground Station
extern uint8_t remoteip[];

/// \brief IP of this on-board-computer
extern uint8_t ip[];

/// \brief MAC address of this on-board-computer
extern uint8_t MAC[];

/// \brief UDP port of this obc
extern uint16_t port;

/// \brief Remote UDP port of the Ground Station
extern uint16_t port2;

/// \brief Helper counter
extern uint8_t counter;

/// \brief Message-to-transmit buffer
extern char message[];

/// \brief Received-message buffer
extern uint8_t received[];

/// \brief Helper for receiving
extern uint16_t received_size;

/// \brief Random packet for debugging
extern uint8_t packet[];

/// \brief UDP packet-to-send buffer
extern uint8_t udp_buffer[];

/// \brief Read MAC address of the ENC28J60 module and send through UART
void display_mac();

/// \brief Send random packet to remote host for testing purposes
void send_random_packet();

/// \brief Send received packet through UART for debugging purposes
void display_received_packet(uint16_t size, uint8_t* packet);

/// \brief Returns non-zero if SYN flag is set
uint8_t is_tcp_syn(uint8_t* packet); // check if tcp packet is a syn packet; 0 if is not a syn, 1 is it is

/// \brief Returns non-zero if both SYN and ACKflag is set
uint8_t is_tcp_synack(uint8_t* packet);

/// \brief Returns non-zero if ACK flag is set
uint8_t is_tcp_ack(uint8_t* packet);

/// \brief Returns non-zero if FIN flag is set
uint8_t is_tcp_fin(uint8_t* packet);

/// \brief Returns non-zero if PSH flag is set
uint8_t is_tcp_psh(uint8_t* packet);

/// \brief Returns non-zero if RST flag is set
uint8_t is_tcp_rst(uint8_t* packet);

/// \brief Returns non-zero if both PSH and ACK flag is set
uint8_t is_tcp_pshack(uint8_t* packet);

/// \brief Returns non-zero if both FIN and ACK flag is set
uint8_t is_tcp_finack(uint8_t* packet); // end transmission request

/// \brief Returns non-zero if both FIN, PSH and ACK flag is set
uint8_t is_tcp_finpshack(uint8_t* packet); // end transmission request

/// \brief End transmission, close connection if remote host wants to
uint8_t tcp_fin_handler(uint8_t* packet); // end transmission handler

/// \brief Validate received ethernet packet
uint8_t eth_pktrecv_valid();

/// \brief Handle received packet. Send response to ping and ARP, parse TCP commands, discard other
uint8_t eth_packet_handler(uint8_t* packet, uint16_t packet_size);

///  \brief returns packet ID
uint16_t eth_pktid(uint8_t* packet);

/// \brief returns non-zero if packett's id is valid
uint8_t tcp_packet_id_valid(uint16_t packetid);

/// \brief send packet to remote host
uint8_t udp_send(uint8_t* packet, uint16_t length);

/** @} */
#endif /* ETH_LUSTRO_H_ */
