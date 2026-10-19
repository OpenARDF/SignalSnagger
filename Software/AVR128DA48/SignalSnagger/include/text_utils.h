#ifndef SIGNALSNAGGER_TEXT_UTILS_H_
#define SIGNALSNAGGER_TEXT_UTILS_H_

#include <stddef.h>

bool only_digits(const char* text);
bool isDisplayable(const char* text, size_t maximumLength);

#endif /* SIGNALSNAGGER_TEXT_UTILS_H_ */
