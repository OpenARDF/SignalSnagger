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
#include <stddef.h>

class CircularStringBuff {
  public:
    ~CircularStringBuff();
    CircularStringBuff(size_t);
    /* This class owns a malloc-backed buffer; copying would double-free it. */
    CircularStringBuff(const CircularStringBuff&) = delete;
    CircularStringBuff& operator=(const CircularStringBuff&) = delete;

    void put(char item);
    /** Queue a complete null-terminated string without a partial write. Returns true on failure. */
    bool putString(const char* string);
    char get(void);
    /** Copy one framed string. Returns true on failure and does not consume it. */
    bool getString(char* str, size_t capacity, size_t *length);
    void reset(void);
    bool empty(void) const;
    bool full(void) const;
    size_t capacity(void) const;
    size_t size(void) const;
	size_t remaining(void) const;
	char pop(void);
	void setBusy(bool busy);
	bool isBusy(void);

  private:
	int head_;
	int tail_;
	bool full_;
	char* buf_;
	size_t max_size_;
	volatile bool busy_;
};
