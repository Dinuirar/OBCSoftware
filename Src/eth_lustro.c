#include "eth_lustro.h"
#include "lustro_config.h"
#include <string.h>

extern void NVIC_SystemReset();

uint8_t remoteMAC[] = { 0x90, 0x48, 0x9a, 0xb8, 0x7a, 0x8d };
uint8_t remoteip[] = { 192, 168, 1, 136 };
uint16_t port2 = 43;
uint8_t MAC[] = { 0x20, 0x37, 0x09, 0x11, 0x42, 0x89 };
uint8_t ip[] = { 192, 168, 1, 42 };
uint16_t port = 43879;

uint8_t counter = 0x00;

char message[MES_BUF_SIZE];
uint8_t udp_buffer[UDP_BUF_SIZE];
uint8_t received[REC_BUF_SIZE];
uint16_t received_size = 0;

uint8_t packet[] = {
		  0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xff, 0xff, 0x55, 0x65, 0x78,
		  0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xff, 0xff, 0x55, 0x65, 0x78,
		  0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xff, 0xff, 0x55, 0x65, 0x78,
		  0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xff, 0xff, 0x55, 0x65, 0x78
};

void display_mac() {
//	counter++;
	uint8_t mac_read[6];
	mac_read[0] = enc28j60Read( MAADR5 );
	mac_read[1] = enc28j60Read( MAADR4 );
	mac_read[2] = enc28j60Read( MAADR3 );
	mac_read[3] = enc28j60Read( MAADR2 );
	mac_read[4] = enc28j60Read( MAADR1 );
	mac_read[5] = enc28j60Read( MAADR0 );
	sprintf(message, "hello, i'm %02x:%02x:%02x:%02x:%02x:%02x\n\r", mac_read[0], mac_read[1], mac_read[2], mac_read[3], mac_read[4], mac_read[5]);
	uart_send(message);
}
void send_random_packet() {
	uint16_t len = 500;
	uint8_t packet[500];
	for (uint16_t i = 0; i < len; i++) {
		packet[i] = i;
	}
	enc28j60PacketSend(500, packet);
}
void display_received_packet(uint16_t size, uint8_t* packet) {
	char message[200];
	uart_send("packet: \n\r");
	for (uint16_t i = 0; i < size; i++) {
		strcpy(message, "");
		sprintf(message, "%02x ", packet[i]);
		uart_send( message );
		if ( (i+1) % 16 == 0)
			uart_send( "\n\r" );
		else if ( (i+1) % 8 == 0 )
			uart_send( "  " );
	}
	uart_send("\n\r \n\r");
}

const uint8_t FLAG_CWR = 128;
const uint8_t FLAG_ECE = 64;
const uint8_t FLAG_URG = 32;
const uint8_t FLAG_ACK = 16;
const uint8_t FLAG_PSH = 8;
const uint8_t FLAG_RST = 4;
const uint8_t FLAG_SYN = 2;
const uint8_t FLAG_FIN = 1;

uint8_t is_tcp_syn(uint8_t* packet) {
	return packet[47] & FLAG_SYN;
}
uint8_t is_tcp_synack(uint8_t* packet) {
	return (packet[47] & FLAG_SYN) && (packet[47] & FLAG_ACK);
}
uint8_t is_tcp_ack(uint8_t* packet) {
	return packet[47] & FLAG_ACK;
}
uint8_t is_tcp_fin(uint8_t* packet) {
	return packet[47] & FLAG_FIN;
}
uint8_t is_tcp_psh(uint8_t* packet) {
	return packet[47] & FLAG_PSH;
}
uint8_t is_tcp_pshack(uint8_t* packet) { // tcp keep-alive
	return (packet[47] & FLAG_PSH) && (packet[47] & FLAG_ACK);
}
uint8_t is_tcp_finack(uint8_t* packet) { // end transmission request
	return (packet[47] & FLAG_FIN) && (packet[47] & FLAG_ACK);
}
uint8_t is_tcp_finpshack(uint8_t* packet) { // end transmission request
	return (packet[47] & FLAG_FIN) && (packet[47] & FLAG_PSH) && (packet[47] & FLAG_ACK);
}
uint8_t is_tcp_rst(uint8_t* packet) {
	return (packet[47] & FLAG_RST);
}
uint8_t tcp_fin_handler(uint8_t* packet) {
	make_tcp_ack_from_any(packet, 0, 0);
	return 1;
}
uint8_t eth_pktrecv_valid() {
	static uint16_t prev_reg_value;
	uint16_t reg_value = enc28j60Read( EPKTCNT );
	if (reg_value == prev_reg_value)
		return 0;
	prev_reg_value = reg_value;
	return 1;
}
uint8_t tcp_packet_id_valid(uint16_t packetid) {
	static uint16_t prev_id;
	//TODO: validate id here
	prev_id = packetid;
	return 0;
}

uint8_t command_handler(uint8_t command, uint8_t arg1, uint8_t arg2) {
	switch (command) {

	case SET_SPEED:
		motor_speed = arg1;
		sprintf(message, "motors' speed set to %d\n\r");
		uart_send(message);
		return SET_SPEED;

	case DOWNSTREAM:
		if (arg1 == 0) {
			if ( arg2 != 0) {
				downstream_enable = 1;
				//				downstream_interval = arg2;
				uart_send("downstream enabled\n\r");
			}
			else {
				downstream_enable = 0;
				uart_send("downstream disabled\n\r");
			}
		}
		return DOWNSTREAM;

	case SET_DOWNSTREAM_INTERVAL:
		if ( arg1 == 0 ) {
			downstream_enable = 0;
			uart_send("downstream disabled\n\r");
		}
		downstream_interval = arg1;
		return SET_DOWNSTREAM_INTERVAL;

	case SUDO_STOPM:
		motor_enable = 0;
		uart_send("motor disabled\n\r");
		return SUDO_STOPM;

	case SUDO_RUNM:
		motor_enable = 1;
		uart_send("motor enabled\n\r");
		return SUDO_RUNM;

	case SUDO_SET_MODE:
		if (arg1 == SCANNING || arg1 == IDLE || arg1 == MANUAL) {
			status = arg1;
			uart_send("mode set\n\r");
		}
		else {
			uart_send("unknown mode\n\r");
		}
		return SUDO_SET_MODE;

	case SUDO_RESET:
		uart_send("reset!\n\r");
		NVIC_SystemReset();
		return 0;
	}
	uart_send( "command unknown\n\r" );
	return 0;
}

// TODO: check for defines or add own defines for TCP packet types
// returnVal recPacketType
// 1 ARP
// 2 Ping
// 3 SYN
// 4 FIN
// 5 PSH-ACK -> keep alive
// 6 ACK
// 0 eth packet not for us
uint8_t eth_packet_handler(uint8_t* received, uint16_t received_size) {
	static uint16_t next_expected_id = 0;
	static uint16_t prev_id = 0;
	uint16_t packet_id = received[18] << 8 | received[19];
	uint16_t received_len = 0;
	// ARP reply (IP and MAC resolution)
	if ( eth_type_is_arp_and_my_ip(received, received_size) ) {
		make_arp_answer_from_request( received );
		return 1;
	}
	// ping reply
	if ( received[IP_PROTO_P] == IP_PROTO_ICMP_V && received[ICMP_TYPE_P] == ICMP_TYPE_ECHOREQUEST_V ) {
		make_echo_reply_from_request(received, received_size);
		return 2;
	}
	// if this packet is not for us - return 0
	if ( !eth_type_is_ip_and_my_ip(received, received_size) ) {
		return 0;
	}
	if ( is_tcp_syn(received) ) {
		make_tcp_synack_from_syn( received );
		return 3;
	}
	else if (is_tcp_fin(received) ) {
		if( tcp_fin_handler(received) ) {
			return 4;
		}
		return 4;
	}
	else if ( is_tcp_pshack(received) ) {
//		make_tcp_ack_from_any(received, 0, 0);
//		return 5;
	}
	else if ( is_tcp_ack(received) ) {
//		make_tcp_ack_from_any(received, 0, 0);
	}
	if ( is_tcp_psh(received) ) { // then we have a connection and the incoming packet is data
		// check ack number, check next expected ack number;
		// if not equal
		//     if one less - return 5 (keep-alive packet)
		//     else return 7 ("incorrect packet id\n\r")
		if (packet_id == prev_id + 1) { // received packet is keep-alive, so discard it
			make_tcp_ack_from_any(received, 0, 0);
			return 5;
		}
		received_len = received_size - 54;
		make_tcp_ack_from_any(received, 0, 0);

		next_expected_id = packet_id + received_len;
		prev_id = packet_id;
		uint8_t received_number, received_argument, received_second_argument;
		received_number = received[TCP_DATA_P + LUSTRO_DATA_OFFSET];
		received_argument = received[TCP_DATA_P + LUSTRO_DATA_OFFSET + 1];
		received_second_argument = received[TCP_DATA_P + LUSTRO_DATA_OFFSET + 2];

		strcpy(message, "");
		sprintf(message, "id=0x%04X next=%04X len=%d command: 0x%02X, arg1: 0x%02X, arg2: 0x%02X\n\r",
				packet_id, next_expected_id, received_len,
				received_number, received_argument, received_second_argument);
		uart_send( message );

		command_handler(received_number, received_argument, received_second_argument);

		return 6;
	}
	return 10;
}

uint16_t eth_pktid(uint8_t* packet) {
	return (packet[TCP_ID_P_H] << 8) | packet[TCP_ID_P_L];
}

uint8_t udp_send(uint8_t* packet, uint16_t length) {
	for (uint8_t i = 0; i < length; i++) {
		if ( UDP_DATA_P + i >= UDP_BUF_SIZE ) {
			ES_send_udp_data2(udp_buffer, remoteMAC, UDP_BUF_SIZE - UDP_DATA_P, port, remoteip, port2);
			return 0;
		}
		udp_buffer[UDP_DATA_P+i] = packet[i];
	}
	ES_send_udp_data2(udp_buffer, remoteMAC, length, port, remoteip, port2);
	return 1;
}
