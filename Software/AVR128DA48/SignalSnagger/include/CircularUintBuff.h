/* 
* CircularUintBuff.h
*
* Created: 8/21/2024 10:12:31 AM
* Author: charl
*/


#ifndef __CIRCULARUINTBUFF_H__
#define __CIRCULARUINTBUFF_H__

#include <stddef.h>
#include "defs.h"

class CircularUintBuff {
	public:
	~CircularUintBuff();
	CircularUintBuff(size_t);

	void put(uint16_t item);
	float get(void);
	void reset(void);
	bool empty(void) const;
	bool full(void) const;
	size_t capacity(void) const;
	size_t size(void) const;
	size_t remaining(void) const;
	uint16_t pop(void);
	void setBusy(bool busy);
	bool isBusy(void);
	uint16_t olympic(void);

	public:
	int head_;
	int tail_;
	bool full_;
	uint16_t* buf_;
	size_t max_size_;
	volatile bool busy_;
};


#endif //__CIRCULARUINTBUFF_H__
