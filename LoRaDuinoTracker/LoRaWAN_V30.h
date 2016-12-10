/******************************************************************************************
*
* File:        LoRaWAN_V30.h
* Author:      Gerben den Hartog
* Compagny:    Ideetron B.V.
* Website:     http://www.ideetron.nl/LoRa
* E-mail:      info@ideetron.nl
******************************************************************************************/

/****************************************************************************************
*
* Created on: 			  20-11-2015
* Supported Hardware:	ID150119-02 Nexus board with RFM95
*
* History:
*
* Firmware version: 1.0
* First version for LoRaWAN
* 
* Firmware version: 2.0
* Ported to Arduino using own AES encryption
* 
* Firmware version 3.0
* Added receiving in receive slot 2
****************************************************************************************/

#ifndef LORAWAN_V30_H
#define LORAWAN_V30_H

/*
*****************************************************************************************
* DEFINES
*****************************************************************************************
*/
/*
#define DS2401  2
#define MFP     3
#define DIO0    4
#define DIO1    5 
#define DIO5    6
#define DIO2    7
#define CS      8
#define LED     9
*/

//aangepast door rein voor arduino shield
#define DS2401  A1
#define MFP     A2
#define DIO0    2
#define DIO1    3 
#define DIO2    4
#define DIO3    5 //unused
#define DIO4    6 //unused
#define DIO5    7
#define CS      8
#define LED     9
#define GPSRX   9
//#define GPSTX   5

#endif
