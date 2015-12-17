#ifndef _CONFIG_H_
#define _CONFIG_H_
//
#include <msp430.h>
//

// Anzahl der Aktuatoren im Funknetz
#define ACTUATOR 3

#define ENTRY_LEN 32



//SPI PINS
#define SCLK_PIN			BIT4	// 1.4
#define MISO_PIN			BIT2	// 1.2
#define MOSI_PIN			BIT3	// 1.3
#define CS_PIN				BIT7

#define SPI_OUT_PORT		P1OUT
#define SPI_IN_PORT			P1IN
#define SPI_SCLK_PORT		P1OUT
#define SPI_SEL_PORT		P1SEL
#define SPI_DIR_PORT		P1DIR

/*
 *  Serielle PINS
 */
#define UART_TX_PIN			BIT6	//1.6
#define UART_RX_PIN			BIT5	//1.5
#define UART_SEL_PORT		P1SEL
#define UART_DIR_PORT		P1DIR

/*
 * SD-Card PINS
 */
#define SD_SCS_PIN			BIT7	// 1.7
#define SD_SCS_PORT			P1OUT
//
#define SD_NPWR_PIN			BIT7
#define SD_NPWR_PORT		P3OUT

/*
 * W5200 PINS
 */
#define WIZ_INT_PIN				BIT0 // 3.0
#define WIZ_INT_PORT			P3IE
#define WIZ_PWDN_PIN			BIT1 // 3.1
#define WIZ_PWDN_PORT			P3OUT
#define WIZ_RESET_PIN			BIT2 // 3.2
#define WIZ_RESET_PORT			P3OUT
#define WIZ_SCS_PIN				BIT0 // 1.0
#define WIZ_SCS_PORT			P1OUT
// port masks, used to prevent accidental configuration changes
#define P1MASK					0// add all P1 pins
#define P2MASK					WIZ_INT_PIN + WIZ_PWDN_PIN + WIZ_RESET_PIN + WIZ_SCS_PIN// add all P2 pins
#define P3MASK					SCLK_PIN + MISO_PIN + MOSI_PIN // add all P3 pins
//
#define P3DIR_SETUP P3DIR |= SCLK_PIN + MOSI_PIN
#define P3OUT_SETUP P3OUT &= ~SCLK_PIN; P3OUT |= MOSI_PIN
#define P2DIR_SETUP P2DIR |=  WIZ_PWDN_PIN + WIZ_RESET_PIN + WIZ_SCS_PIN
#define P2OUT_SETUP P2OUT |=  WIZ_RESET_PIN + WIZ_SCS_PIN; P2OUT &= ~WIZ_PWDN_PIN


/*
 * SD Config.
 */
#define sdSelect()			SD_SCS_PORT &= ~SD_SCS_PIN	/* CS = L     P1.0*/
#define	sdDeselect()		SD_SCS_PORT |= SD_SCS_PIN   /* CS = H     P1.0*/
#define	MMC_SEL				!(SD_SCS_PORT & SD_SCS_PIN)	/* CS status (true:CS == L) */

//

/*
 * Ethernet Config.
 */
#define RESPONSE_TYPE_HTML

#define TX_MAX_BUF_SIZE	0x7F // we can go up to 256, but we would have to decrease RX down to 64, 256/64 (G2553 chip)
#define RX_MAX_BUF_SIZE	0x7F //


#define WIZ_RESET_0				WIZ_RESET_PORT &= ~WIZ_RESET_PIN
#define WIZ_RESET_1				WIZ_RESET_PORT |= WIZ_RESET_PIN
#define WIZ_POWER_UP			WIZ_PWDN_PORT &= ~WIZ_PWDN_PIN
#define WIZ_POWER_DOWN			WIZ_PWDN_PORT |= WIZ_PWDN_PIN
#define WIZ_SELECT				WIZ_SCS_PORT &= ~WIZ_SCS_PIN
#define WIZ_DESELECT			WIZ_SCS_PORT |= WIZ_SCS_PIN
#define WIZ_IE_ENABLE			WIZ_INT_PORT |= WIZ_INT_PIN
#define WIZ_IE_DISABLE			WIZ_INT_PORT &= ~WIZ_INT_PIN
//
#define wizPowerUp()			WIZ_POWER_UP
#define wizPowerDown()			WIZ_POWER_DOWN
#define wizSelect()				WIZ_SELECT
#define wizDeselect()			WIZ_DESELECT
#define wizEnableInterrupt()	WIZ_IE_ENABLE
#define wizDisableInterrupt()	WIZ_IE_DISABLE
//


/*
 * SD Defines
 */
/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define	ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

/* Card type flags (CardType) */
#define CT_MMC				0x01	/* MMC ver 3 */
#define CT_SD1				0x02	/* SD ver 1 */
#define CT_SD2				0x04	/* SD ver 2 */
#define CT_BLOCK			0x08	/* Block addressing */



/*
 * W5200 Defines
 */
#define SOCK_CONFIG				2	// UDP
#define SOCK_DNS				2	// UDP
#define SOCK_DHCP				3	// UDP
#define MAX_BUF_SIZE			1460
#define KEEP_ALIVE_TIME			30	// 30 sec
#define	MAX_SOCK_NUM			8
//
#define WINDOWFULL_FLAG_ON 		1
#define WINDOWFULL_FLAG_OFF 	0
#define WINDOWFULL_MAX_RETRY_NUM 3
#define WINDOWFULL_WAIT_TIME 	1000
#define COMMON_BASE 			0x0000
//
#define REQ_ACTION				0x01
#define REQ_TARGET				0x02
#define REQ_NUMBER				0x03
#define REQ_VALUE				0x04
#define PARAM_NAME				0x06
#define PARAM_VALUE				0x07
#define REQ_PAGE				0x08
//
#define MR						(COMMON_BASE + 0x0000)
#define GAR0			        (COMMON_BASE + 0x0001)	/* Gateway IP Register address */
#define SUBR0			        (COMMON_BASE + 0x0005)	/* Subnet mask Register address */
#define SHAR0					(COMMON_BASE + 0x0009)	/* Source MAC Register address */
#define SIPR0					(COMMON_BASE + 0x000F)	/* Source IP Register address */
#define IR						(COMMON_BASE + 0x0015)	/* Interrupt Register */
#define IR2						(COMMON_BASE + 0x0034)	/* Socket Interrupt Register */
#define PHY						(COMMON_BASE + 0x0035)	/* PHY Status Register */
#define IMR						(COMMON_BASE + 0x0036)	/* Interrupt mask register */
#define IMR2					(COMMON_BASE + 0x0016)	/* Socket Interrupt Mask Register */
#define RTR						(COMMON_BASE + 0x0017)	/* Timeout register address( 1 is 100us ) */
#define RCR						(COMMON_BASE + 0x0019)	/* Retry count register */
#define PATR0			        (COMMON_BASE + 0x001C)	/* Authentication type register address in PPPoE mode */
#define PPPALGO					(COMMON_BASE + 0x001E)
#define PTIMER					(COMMON_BASE + 0x0028)	/* PPP LCP Request Timer register  in PPPoE mode */
#define PMAGIC					(COMMON_BASE + 0x0029)	/* PPP LCP Magic number register  in PPPoE mode */
#define VERSIONR				(COMMON_BASE + 0x001F)	/* chip version register address */
#define UIPR0					(COMMON_BASE + 0x002A)	/* Unreachable IP register address in UDP mode */
#define UPORT0			        (COMMON_BASE + 0x002E)	/* Unreachable Port register address in UDP mode */
#define INTLEVEL0				(COMMON_BASE + 0x0030)	/* set Interrupt low level timer register address */
#define INTLEVEL1				(COMMON_BASE + 0x0031)
#define CH_BASE					(COMMON_BASE + 0x4000)	/* socket register */
#define CH_SIZE					0x0100					/* size of each channel register map */
#define Sn_MR(ch)				(CH_BASE + ch * CH_SIZE + 0x0000)	/* socket Mode register */
#define Sn_CR(ch)				(CH_BASE + ch * CH_SIZE + 0x0001)	/* channel Sn_CR register */
#define Sn_IR(ch)				(CH_BASE + ch * CH_SIZE + 0x0002)	/* channel interrupt register */
#define Sn_SR(ch)				(CH_BASE + ch * CH_SIZE + 0x0003)	/* channel status register */
#define Sn_PORT0(ch)			(CH_BASE + ch * CH_SIZE + 0x0004)	/* source port register */
#define Sn_DHAR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0006)	/* Peer MAC register address */
#define Sn_DIPR0(ch)			(CH_BASE + ch * CH_SIZE + 0x000C)	/* Peer IP register address */
#define Sn_DPORT0(ch)			(CH_BASE + ch * CH_SIZE + 0x0010)	/* Peer port register address */
#define Sn_MSSR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0012)	/* Maximum Segment Size(Sn_MSSR0) register address */
#define Sn_PROTO(ch)			(CH_BASE + ch * CH_SIZE + 0x0014)	/* Protocol of IP Header field register in IP raw mode */
#define Sn_TOS(ch)				(CH_BASE + ch * CH_SIZE + 0x0015)	/* IP Type of Service(TOS) Register */
#define Sn_TTL(ch)				(CH_BASE + ch * CH_SIZE + 0x0016)	/* IP Time to live(TTL) Register */
#define Sn_RXMEM_SIZE(ch)		(CH_BASE + ch * CH_SIZE + 0x001E)	/* Receive memory size register */
#define Sn_TXMEM_SIZE(ch)		(CH_BASE + ch * CH_SIZE + 0x001F)	/* Transmit memory size register */
#define Sn_TX_FSR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0020)	/* Transmit free memory size register */
#define Sn_TX_RD0(ch)			(CH_BASE + ch * CH_SIZE + 0x0022)	/* Transmit memory read pointer register address */
#define Sn_TX_WR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0024)	/* Transmit memory write pointer register address */
#define Sn_RX_RSR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0026)	/* Received data size register */
#define Sn_RX_RD0(ch)			(CH_BASE + ch * CH_SIZE + 0x0028)	/* Read point of Receive memory */
#define Sn_RX_WR0(ch)			(CH_BASE + ch * CH_SIZE + 0x002A)	/* Write point of Receive memory */
#define Sn_IMR(ch)				(CH_BASE + ch * CH_SIZE + 0x002C)	/* socket interrupt mask register */
#define Sn_FRAG(ch)				(CH_BASE + ch * CH_SIZE + 0x002D)	/* frag field value in IP header register */
#define Sn_KEEP_TIMER(ch)		(CH_BASE + ch * CH_SIZE + 0x002F)	/* Keep Timer register */
//
#define MR_RST					0x80		/* reset */
#define MR_WOL					0x20		/* Wake on Lan */
#define MR_PB					0x10		/* ping block */
#define MR_PPPOE				0x08		/* enable pppoe */
#define MR_LB  			        0x04		/* little or big endian selector in indirect mode */
#define MR_AI					0x02		/* auto-increment in indirect mode */
#define MR_IND					0x01		/* enable indirect mode */
//
#define IR_CONFLICT				0x80		/* check ip confict */
#define IR_UNREACH				0x40		/* get the destination unreachable message in UDP sending */
#define IR_PPPoE				0x20		/* get the PPPoE close message */
#define IR_MAGIC				0x10		/* get the magic packet interrupt */
#define IR_SOCK(ch)				(0x01 << ch)/* check socket interrupt */
//
#define Sn_MR_CLOSE				0x00		/* unused socket */
#define Sn_MR_TCP				0x01		/* TCP */
#define Sn_MR_UDP				0x02		/* UDP */
#define Sn_MR_IPRAW				0x03		/* IP LAYER RAW SOCK */
#define Sn_MR_MACRAW	        0x04		/* MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE				0x05		/* PPPoE */
#define Sn_MR_ND				0x20		/* No Delayed Ack(TCP) flag */
#define Sn_MR_MULTI				0x80		/* support multicating */
//
#define Sn_CR_OPEN				0x01		/* initialize or open socket */
#define Sn_CR_LISTEN			0x02		/* wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT	        0x04		/* send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON			0x08		/* send closing reqeuset in tcp mode */
#define Sn_CR_CLOSE				0x10		/* close socket */
#define Sn_CR_SEND				0x20		/* update txbuf pointer, send data */
#define Sn_CR_SEND_MAC	        0x21		/* send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP     	0x22		/*  send keep alive message */
#define Sn_CR_RECV				0x40		/* update rxbuf pointer, recv data */
//
#define Sn_IR_SEND_OK			0x10		/* complete sending */
#define Sn_IR_TIMEOUT			0x08		/* assert timeout */
#define Sn_IR_RECV				0x04		/* receiving data */
#define Sn_IR_DISCON			0x02		/* closed socket */
#define Sn_IR_CON				0x01		/* established connection */
//
#define SOCK_CLOSED				0x00		/* closed */
#define SOCK_INIT 				0x13		/* init state */
#define SOCK_LISTEN				0x14		/* listen state */
#define SOCK_SYNSENT	   		0x15		/* connection state */
#define SOCK_SYNRECV		   	0x16		/* connection state */
#define SOCK_ESTABLISHED		0x17		/* success to connect */
#define SOCK_FIN_WAIT			0x18		/* closing state */
#define SOCK_CLOSING		   	0x1A		/* closing state */
#define SOCK_TIME_WAIT			0x1B		/* closing state */
#define SOCK_CLOSE_WAIT			0x1C		/* closing state */
#define SOCK_LAST_ACK			0x1D		/* closing state */
#define SOCK_UDP				0x22		/* udp socket */
#define SOCK_IPRAW				0x32		/* ip raw mode socket */
#define SOCK_MACRAW				0x42		/* mac raw mode socket */
#define SOCK_PPPOE				0x5F		/* pppoe socket */
//
#define IPPROTO_IP              0           /* Dummy for IP */
#define IPPROTO_ICMP            1           /* Control message protocol */
#define IPPROTO_IGMP            2           /* Internet group management protocol */
#define IPPROTO_GGP             3           /* Gateway^2 (deprecated) */
#define IPPROTO_TCP             6           /* TCP */
#define IPPROTO_PUP             12          /* PUP */
#define IPPROTO_UDP             17          /* UDP */
#define IPPROTO_IDP             22          /* XNS idp */
#define IPPROTO_ND              77          /* UNOFFICIAL net disk protocol */
#define IPPROTO_RAW             255         /* Raw IP packet */
//

// UART commands
#define CMD_MOIS 'A'
#define CMD_VOLT 'B'
#define CMD_MIN  'C'
#define CMD_MAX  'D'
#define CMD_CALI 'E'
#define CMD_DRY  'F'
#define CMD_WET  'G'
#define CMD_FIN  'H'
#define CMD_TEST 'I'
#define CMD_VERS 'J'
#define CMD_RSSI 'K'
#define CMD_ERROR 'Z'



#endif

