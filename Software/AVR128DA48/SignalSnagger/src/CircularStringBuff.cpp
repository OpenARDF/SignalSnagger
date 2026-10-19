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
#include "CircularStringBuff.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>

CircularStringBuff::CircularStringBuff(size_t size)
{
  buf_ = (char*)malloc(size);
  max_size_ = size;
  head_ = 0;
  tail_ = 0;
  full_ = false;
  busy_ = false;
}

CircularStringBuff::~CircularStringBuff() {
	free(buf_);
}


void CircularStringBuff::reset()
{
  head_ = tail_;
  full_ = false;
  busy_ = false;
}

bool CircularStringBuff::empty() const
{
  /*if head and tail are equal, we are empty */
  return (!full_ && (head_ == tail_));
}

void CircularStringBuff::setBusy(bool busy)
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

bool CircularStringBuff::isBusy(void)
{
	return busy_;
}

bool CircularStringBuff::full() const
{
  return (full_);
}

size_t CircularStringBuff::capacity() const
{
  return (max_size_);
}

size_t CircularStringBuff::size() const
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

size_t CircularStringBuff::remaining() const
{
	size_t space = 0;

	if (!full_)
	{
		space = capacity() - size();
	}

	return(space);
}

/** 
 * Place another item in the buffer
 */
void CircularStringBuff::put(char item)
{
  if (!buf_ || !max_size_)
  {
    return;
  }

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
char CircularStringBuff::pop()
{
  if (empty())
  {
	  return ('\0');
  }

  /* head_ identifies the next write slot, so move back before reading. */
  if(head_) 
  {
	  head_--;
  }
  else
  {
	  head_ = (max_size_-1);
  }
  char val = buf_[head_];
  
  full_ = false;

  return (val);
}

/** 
 * Return the FIFO entry and delete it from the buffer
 */
char CircularStringBuff::get()
{
  if (empty())
  {
    return ('\0');
  }

  /*Read data and advance the tail (we now have a free space) */
  char val = buf_[tail_];
  full_ = false;
  tail_ = (tail_ + 1) % max_size_;

  return (val);
}

bool CircularStringBuff::putString(const char* string)
{
	bool fail = true;
	
	if(!string || !buf_ || !max_size_ || full_) return(fail);

	size_t size = strlen(string);
	/* Every queued string owns a terminator; reject before mutating on overflow. */
	if(size >= remaining()) return(fail);

	size_t i = 0;
	
	while(i < size)
	{
		put(string[i++]);
	}
	
	put('\0');
	
	return(false);
}

bool CircularStringBuff::getString(char* str, size_t capacity, size_t *length)
{
	if(length) *length = 0;
	if(!str || !capacity || !buf_)
	{
		return true;
	}

	if(empty())
	{
		str[0] = '\0';
		return false;
	}

	/* Measure the next framed string before consuming anything. */
	size_t available = size();
	size_t stringLength = 0;
	int index = tail_;
	while(stringLength < available && buf_[index] != '\0')
	{
		++stringLength;
		index = (index + 1) % max_size_;
	}

	if(stringLength == available || stringLength + 1 > capacity)
	{
		return true;
	}

	for(size_t i = 0; i < stringLength; ++i)
	{
		str[i] = get();
	}
	(void)get(); /* Consume the framing terminator. */
	str[stringLength] = '\0';
	if(length) *length = stringLength;
	return false;
}
