#ifndef NRF_COMMANDS_H_
#define NRF_COMMANDS_H_

/* nRF Commands Specs.
 * There are two kinds of commands: One-Shot commands and multi-shot.
 *
 * One-shot commands: 	these are commands that will execute a particular task on the nRF just after the command
 * 						is received.
 * Multi-shot commands: these commands will always start with a One-shot command that will inform the nRF that
 * 						we will send more data. After sending the first one shot command, the nRF will expect
 * 						to receive the size of the data to be transmitted in the next transmission. Finally,
 * 						after these two first bytes of commands, you can start putting your payload.
 *
 * 						ex:
 *
 * 						send_spi(_SEND_OVER_Bt);
 * 						send_spi(SIZE_DATA);
 * 						send_spi(ALL_THE_PAYLOAD_VECTOR)
 * */

/*One shot commands - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* General Commands to control stuff using the nordic*/
#define _ENABLE_ANALOG					0x55
#define _DISABLE_ANALOG					0x50
#define _LEAVING_MASTER_MODE            0XF0

#define _SET_BLE_BUFFER_NORDIC          0x36
#define _RETRIEVE_BLE_PCK_SIZE          0X37
#define _RETRIEVE_BLE_PCK               0x38
#define _SEND_OVER_NUS                  0x69

#define _ENQ_BATTERY_VOLTAGE_VALUE    	0x14
#define _ENQ_8_RC_CHANNELS              0x15

/* Led functions */
#define _LED_1_ON                       0x21
#define _LED_1_OFF                      0x22
#define _LED_2_ON                       0x25
#define _LED_2_OFF                      0x26

/*Multi-shot commands - - - - - - - - - - - - - - - - - - - - - - - - -*/
#define	_SEND_OVR_BT_SINGLE				0x66
#define _ENABLE_BLE						0x35

#define BLE_UART_MSG                    0x40
#define BLE_NO_MSG                      0x41

/*SPI PACKET IDENT DEFINES*/
/*SPI packets will be defined*/













#endif
