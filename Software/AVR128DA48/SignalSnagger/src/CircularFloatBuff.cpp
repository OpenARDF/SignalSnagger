/**********************************************************************************************
    Copyright © 2019 Digital Confections LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in the
    Software without restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so, subject to the
    following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
    PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
    FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
    OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

**********************************************************************************************/
#include "CircularFloatBuff.h"
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include "defs.h"

CircularFloatBuff::CircularFloatBuff(size_t size)
{
  buf_ = (float*)malloc(size*sizeof(float));
  max_size_ = size;
  head_ = 0;
  tail_ = 0;
  full_ = false;
  busy_ = false;
}

CircularFloatBuff::~CircularFloatBuff() {
	free(buf_);
}


void CircularFloatBuff::reset()
{
  head_ = tail_;
  full_ = false;
  busy_ = false;
}

bool CircularFloatBuff::empty() const
{
  /*if head and tail are equal, we are empty */
  return (!full_ && (head_ == tail_));
}

void CircularFloatBuff::setBusy(bool busy)
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

bool CircularFloatBuff::isBusy(void)
{
	return busy_;
}

bool CircularFloatBuff::full() const
{
  return (full_);
}

size_t CircularFloatBuff::capacity() const
{
  return (max_size_);
}

size_t CircularFloatBuff::size() const
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

size_t CircularFloatBuff::remaining() const
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
void CircularFloatBuff::put(float item)
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
float CircularFloatBuff::pop()
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
float CircularFloatBuff::get()
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

float CircularFloatBuff::olympic()
{
	float accum=0., max=0., min=0.;
	
	for(uint8_t i = 0; i < max_size_; ++i)
	{
		accum += buf_[i];
		min = MIN(buf_[i], min);
		max = MAX(buf_[i], max);
	}
	  
	accum -= min;
	accum -= max;
	
	return(accum/((float)(max_size_-2)));
}

