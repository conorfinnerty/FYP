/*
 * CloudBrokerCredentials.h
 *
 *  Created on: Nov 11, 2022
 *      Author: nokeeffe
 */

#ifndef INC_CLOUDBROKERCREDENTIALS_H_
#define INC_CLOUDBROKERCREDENTIALS_H_

/*
 * Add host name DNS here
 * For Adafruit use "io.adafruit.com"
 * For Ubidots use "industrial.api.ubidots.com"
 */
//#define CloudBroker_HostName "industrial.api.ubidots.com"
#define CloudBroker_HostName "industrial.api.ubidots.com"

/*
 * Port = "1883" for unencrypted communication
 */
#define CloudBroker_Port "1883"

/*
 * For Adafruit use your username
 * For Ubidots use your API key
 */
#define CloudBroker_Username "BBFF-IPRUyfG3Nh7zGxoYOO7qRyF9HUgWrd" //BBFF-UyuOym4WKWu1BqAwQdQQEE9JcPB1yw

/*
 * For Adafruit use "your active key
 * For Ubidots use an empty password ""
 */
#define CloudBroker_Password ""

/*
 * For Adafruit use a random ID
 * For Ubidots use your Device ID
 */
#define CloudBroker_ClientID "64216795aaee4c001076ea93"


#endif /* INC_CLOUDBROKERCREDENTIALS_H_ */
