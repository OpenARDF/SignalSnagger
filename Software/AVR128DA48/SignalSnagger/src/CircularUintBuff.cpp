/* 
* CircularUintBuff.cpp
*
* Created: 8/21/2024 10:12:31 AM
* Author: charl
*/


#include "CircularUintBuff.h"
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include "defs.h"

CircularUintBuff::CircularUintBuff(size_t size)
{
  buf_ = (uint16_t*)malloc(size*sizeof(uint16_t));
  max_size_ = size;
  head_ = 0;
  tail_ = 0;
  full_ = false;
  busy_ = false;
}

CircularUintBuff::~CircularUintBuff() {
	free(buf_);
}


void CircularUintBuff::reset()
{
  head_ = tail_;
  full_ = false;
  busy_ = false;
}

bool CircularUintBuff::empty() const
{
  /*if head and tail are equal, we are empty */
  return (!full_ && (head_ == tail_));
}

void CircularUintBuff::setBusy(bool busy)
{
	if(busy)
	{
		busy_ = true;
	}
	else
	{
		busy_ = false;
	}
}

bool CircularUintBuff::isBusy(void)
{
	return busy_;
}

bool CircularUintBuff::full() const
{
  return (full_);
}

size_t CircularUintBuff::capacity() const
{
  return (max_size_);
}

size_t CircularUintBuff::size() const
{
	size_t size = max_size_;

	if (!full_)
	{
		if (head_ >= tail_)
		{
			size = head_ - tail_;
		}
		else
		{
			size = max_size_ + head_ - tail_;
		}
	}

	return (size);
}

size_t CircularUintBuff::remaining() const
{
	size_t space = 0;

	if (!full_)
	{
		space = capacity() - size();
	}

	return(space);
}

/** 
 * Place another item in the buffer. If full, the oldest item will be lost.
 */
void CircularUintBuff::put(uint16_t item)
{
  buf_[head_] = item;

  if (full_)
  {
    tail_ = (tail_ + 1) % max_size_;
  }

  head_ = (head_ + 1) % max_size_;

  full_ = head_ == tail_;
}

/** 
 * Return the last put item and remove it from the buffer
 */
uint16_t CircularUintBuff::pop()
{
  if (empty())
  {
	  return ('\0');
  }

  /*Read data and decrement the head (we now have one more free space) */
  if(head_) 
  {
	  head_--;
  }
  else
  {
	  head_ = (max_size_-1);
  }
  
  float val = buf_[head_];
  
  full_ = false;

  return (val);
}

/** 
 * Return the FIFO entry and delete it from the buffer
 */
float CircularUintBuff::get()
{
  if (empty())
  {
    return ('\0');
  }

  /* Read data and advance the tail (we now have a free space) */
  float val = buf_[tail_];
  full_ = false;
  tail_ = (tail_ + 1) % max_size_;

  return (val);
}

uint16_t CircularUintBuff::olympic()
{
	uint32_t accum=0., max=0., min=0.;
	
	for(uint8_t i = 0; i < max_size_; ++i)
	{
		accum += buf_[i];
		min = MIN(buf_[i], min);
		max = MAX(buf_[i], max);
	}
	  
	accum -= min;
	accum -= max;
	
	return(accum/((uint16_t)(max_size_-2)));
}
