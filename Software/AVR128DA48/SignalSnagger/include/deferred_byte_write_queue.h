/*
 * Fixed-capacity queue for foreground byte writes that must be serviced later.
 */

#ifndef SIGNALSNAGGER_DEFERRED_BYTE_WRITE_QUEUE_H_
#define SIGNALSNAGGER_DEFERRED_BYTE_WRITE_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

struct DeferredByteWrite
{
	uint16_t address;
	uint8_t value;
};

template <size_t Capacity>
class DeferredByteWriteQueue
{
public:
	DeferredByteWriteQueue() : size_(0)
	{
		static_assert(Capacity > 0, "deferred byte-write queue requires capacity");
	}

	bool put(uint16_t address, uint8_t value)
	{
		for(size_t i = 0; i < size_; ++i)
		{
			if(entries_[i].address == address)
			{
				/* Keep the original order while retaining only the newest value. */
				entries_[i].value = value;
				return true;
			}
		}

		if(size_ >= Capacity)
		{
			return false;
		}

		entries_[size_].address = address;
		entries_[size_].value = value;
		++size_;
		return true;
	}

	bool front(DeferredByteWrite* write) const
	{
		if(!write || !size_)
		{
			return false;
		}

		*write = entries_[0];
		return true;
	}

	bool removeFront()
	{
		if(!size_)
		{
			return false;
		}

		for(size_t i = 1; i < size_; ++i)
		{
			entries_[i - 1] = entries_[i];
		}
		--size_;
		return true;
	}

	bool empty() const
	{
		return size_ == 0;
	}

	size_t size() const
	{
		return size_;
	}

private:
	DeferredByteWrite entries_[Capacity];
	size_t size_;
};

#endif /* SIGNALSNAGGER_DEFERRED_BYTE_WRITE_QUEUE_H_ */
