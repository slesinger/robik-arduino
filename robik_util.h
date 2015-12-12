/*
 * robik_move.h
 *
 *  Created on: Dec 12, 2015
 *      Author: Honza Slesinger
 * This header file is shared between robik_driver and arduino sketch
 */

#ifndef ROBIK_UTIL_H_
#define ROBIK_UTIL_H_


#define LOG_STR_LEN 150
char charBuf[LOG_STR_LEN + 1];
char strtmp[LOG_STR_LEN + 1];

robik::GenericStatus *_status_msg;

//determine the sign of a value
template <typename type>
type sign(type value) {
	return type((value>0)-(value<0));
}

template <typename type>
type signNZ(type value) {
	return (value < 0) ? -1 : 1;
}

void setup_util(robik::GenericStatus *status_msg) {

  _status_msg = status_msg;
  _status_msg->status_code = (int32_t *) malloc(20 * sizeof(int32_t));


}

void log_string(String str) {

	unsigned int str_length = str.length();

	if (str_length > 0) {

		if (str_length > LOG_STR_LEN)
			str_length = LOG_STR_LEN;

		str.toCharArray(charBuf, str_length + 1);
		_status_msg->log_message = &charBuf[0];
		_status_msg->log_count++;

	}

}

void log_chars(char *str) {

	if (strlen(str) > 0) {
		strcpy(charBuf, str);
		_status_msg->log_message = &charBuf[0];
		_status_msg->log_count++;

	}

}

void add_status_code (int32_t code) {
	if (_status_msg->status_code_length < 20) {
		_status_msg->status_code[_status_msg->status_code_length] = code;
		_status_msg->status_code_length++;
	}
}




#endif /* ROBIK_UTIL_H_ */
