/*
 * ESP8266_HAL.c
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */


#include "UartRingbuffer_multi.h"
#include "ESP8266_HAL.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "stdlib.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern char buforek [64] ;

char buffer[20];


/*****************************************************************************************************************************************/

void ESP_Init (char *SSID, char *PASSWD)
{
	char data[80];

//	Ringbuf_init();
	Uart_sendstring("AT+RST\r\n", device_uart);
	Uart_sendstring("RESETTING.", pc_uart);
	for (int i=0; i<5; i++)
	{
		Uart_sendstring(".", pc_uart);
		HAL_Delay(1000);
	}

	/********* AT **********/
	Uart_flush(device_uart);
	Uart_sendstring("AT\r\n", device_uart);
	while(!(Wait_for("OK\r\n", device_uart)));
	Uart_sendstring("AT---->OK\r\n", pc_uart);


	/********* AT+CWMODE=1 **********/
	Uart_flush(device_uart);
	Uart_sendstring("AT+CWMODE=1\r\n", device_uart);
	while (!(Wait_for("OK\r\n", device_uart)));
	Uart_sendstring("CW MODE---->1\r\n", pc_uart);


	/********* AT+CWJAP="SSID","PASSWD" **********/
	if(*SSID!=0 || *PASSWD!=0){
	Uart_flush(device_uart);
	Uart_sendstring("connecting... to the provided AP\n", pc_uart);
	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_sendstring(data, device_uart);
	while (!(Wait_for("OK\r\n", device_uart)));
	sprintf (data, "Connected to,\"%s\"\r\n", SSID);
	Uart_sendstring(data,pc_uart);
	}

	/********* AT+CIFSR **********/
	Uart_flush(device_uart);
	Uart_sendstring("AT+CIFSR\r\n", device_uart);
	while (!(Wait_for("CIFSR:STAIP,\"", device_uart)));
	while (!(Copy_upto("\"",buffer, device_uart)));
	while (!(Wait_for("OK\r\n", device_uart)));
	int len = strlen (buffer);
	buffer[len-1] = '\0';
	sprintf (data, "IP ADDR: %s\r\n", buffer);
	Uart_sendstring(data, pc_uart);

//	/********* AT+CIPMUX **********/
//	Uart_flush(device_uart);
//	Uart_sendstring("AT+CIPMUX=1\r\n", device_uart);
//	while (!(Wait_for("OK\r\n", device_uart)));
//	Uart_sendstring("CIPMUX---->OK\r\n", pc_uart);
//
//	/********* AT+CIPSERVER **********/
//	Uart_flush(device_uart);
//	Uart_sendstring("AT+CIPSERVER=1,80\r\n", device_uart);
//	while (!(Wait_for("OK\r\n", device_uart)));
//	Uart_sendstring("CIPSERVER---->OK\r\n", pc_uart);

}




int Server_Send (char *str, int Link_ID)
{
	int len = strlen (str);
	char data[80];
	sprintf (data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data, device_uart);
	while (!(Wait_for(">", device_uart)));
	Uart_sendstring (str, device_uart);
	while (!(Wait_for("SEND OK", device_uart)));
	sprintf (data, "AT+CIPCLOSE=5\r\n");
	Uart_sendstring(data, device_uart);
	while (!(Wait_for("OK\r\n", device_uart)));
	return 1;
}

//void Server_Handle (char *str, int Link_ID)
//{
//	char datatosend[1024] = {0};
//	if (!(strcmp (str, "/ledon")))
//	{
//		sprintf (datatosend, Basic_inclusion);
//		strcat(datatosend, LED_ON);
//		strcat(datatosend, Terminate);
//		Server_Send(datatosend, Link_ID);
//	}
//
//	else if (!(strcmp (str, "/ledoff")))
//	{
//		sprintf (datatosend, Basic_inclusion);
//		strcat(datatosend, LED_OFF);
//		strcat(datatosend, Terminate);
//		Server_Send(datatosend, Link_ID);
//	}
//
//	else
//	{
//		sprintf (datatosend, Basic_inclusion);
//		strcat(datatosend, LED_OFF);
//		strcat(datatosend, Terminate);
//		Server_Send(datatosend, Link_ID);
//	}
//
//}

void ConnectToWebsite(char *Hoststr, char *Addstr){

	/********* AT+CIPSTART **********/
char bufferforCIPStart[80];
char bufferforGetData[80];
//clear buffers
memset(bufferforCIPStart, 0, sizeof(bufferforCIPStart));
memset(bufferforGetData, 0, sizeof(bufferforGetData));

							//AT+CIPSTART=\"TCP\",\"aqzera.pl\",80\r\n
sprintf(bufferforCIPStart, "AT+CIPSTART=\"TCP\",\%s\",80\r\n",Hoststr);
							//GET /artur/ HTTP/1.1\r\nHost: www.aqzera.pl\r\n\r\n
sprintf(bufferforGetData, "GET %s HTTP/1.1\r\nHost: www.%s\r\n\r\n",Addstr,Hoststr);

	Uart_flush(device_uart);

	Uart_sendstring("AT+CIPSTART=\"TCP\",\"aqzera.pl\",80\r\n", device_uart); //443 is https, 80 is http
	while(!(Wait_for("CONNECT\r\n", device_uart)));
	Uart_sendstring("AT+CIPSTART = OK\r\n", pc_uart);

	char atcipsend[20];
	uint8_t dlugosc=strlen(bufferforGetData);

	sprintf(atcipsend,"AT+CIPSEND=%d\r\n",dlugosc);

	Uart_sendstring(atcipsend,device_uart);
	while(!(Wait_for("OK\r\n", device_uart)));

	Uart_sendstring("AT+CIPSEND = OK\r\n", pc_uart);
	HAL_Delay(20); //wait for char  >

	Uart_sendstring(bufferforGetData,device_uart);

	//receive WEBSITE HERE
	HAL_Delay(1000); //wait 1 sec for data.


	Uart_sendstring("AT+CIPCLOSE",device_uart); //needed for some reason.

}

//
//int extract(char *string,  char *left,  char *right, char *output)
//{
//    char  *head;
//    char  *tail;
//    size_t length;
//    char  *result;
//
//    if ((string == NULL) || (left == NULL) || (right == NULL))
//        return 0;
//    length = strlen(left);
//    head   = strstr(string, left);
//    if (head == NULL)
//        return 0;
//    head += length;
//    tail  = strstr(head, right);
//    if (tail == 0)
//        return 0;
//    length = tail - head;
//    result = malloc(1 + length);
//    if (result == NULL)
//        return 0;
//    result[length] = '\0';
//
//
//    memcpy(output, head, length);
//    return 1;
//}
