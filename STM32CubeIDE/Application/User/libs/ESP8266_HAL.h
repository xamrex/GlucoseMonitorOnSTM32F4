/*
 * ESP8266_HAL.h
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */

#ifndef INC_ESP8266_HAL_H_
#define INC_ESP8266_HAL_H_


void ESP_Init (char *SSID, char *PASSWD);
void ConnectToWebsite(char *Hoststr, char *Addstr);

int extract(char *string,  char *left,  char *right, char *output);

#endif /* INC_ESP8266_HAL_H_ */
