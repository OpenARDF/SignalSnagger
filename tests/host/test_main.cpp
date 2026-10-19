#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "CircularStringBuff.h"
#include "receiver_frequency.h"

static_assert(!std::is_copy_constructible<CircularStringBuff>::value,
              "CircularStringBuff must not copy its owning buffer pointer");
static_assert(!std::is_copy_assignable<CircularStringBuff>::value,
              "CircularStringBuff must not copy its owning buffer pointer");

namespace {

int failures = 0;

#define CHECK(condition) check((condition), #condition, __FILE__, __LINE__)

void check(bool condition, const char* expression, const char* file, int line)
{
    if (!condition) {
        fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
        ++failures;
    }
}

void testCircularBufferExactFitIncludesTerminator()
{
    CircularStringBuff buffer(4);
    char output[4] = {};
    size_t length = 0;

    CHECK(!buffer.putString("abc"));
    CHECK(buffer.full());
    CHECK(!buffer.getString(output, sizeof(output), &length));
    CHECK(length == 3);
    CHECK(memcmp(output, "abc", 3) == 0);
    CHECK(buffer.empty());
}

void testCircularBufferRejectsSmallDestinationWithoutMutation()
{
    CircularStringBuff buffer(8);
    char output[3] = {};
    size_t length = 99;

    CHECK(!buffer.putString("abc"));
    CHECK(buffer.getString(output, sizeof(output), &length));
    CHECK(length == 0);
    CHECK(buffer.size() == 4);
}

void testCircularBufferRejectsOversizeWithoutMutation()
{
    CircularStringBuff buffer(4);

    CHECK(buffer.putString(0));
    CHECK(buffer.putString("abcd"));
    CHECK(buffer.empty());
    CHECK(buffer.remaining() == 4);
}

void testCircularBufferRejectsUnframedDataWithoutMutation()
{
    CircularStringBuff buffer(4);
    char output[5] = "keep";
    size_t length = 99;

    buffer.put('a');
    buffer.put('b');
    CHECK(buffer.getString(output, sizeof(output), &length));
    CHECK(length == 0);
    CHECK(strcmp(output, "keep") == 0);
    CHECK(buffer.size() == 2);
}

void testCircularBufferPopReturnsMostRecentByte()
{
    CircularStringBuff buffer(4);

    buffer.put('a');
    buffer.put('b');
    CHECK(buffer.pop() == 'b');
    CHECK(buffer.pop() == 'a');
    CHECK(buffer.pop() == '\0');
}

void testCircularBufferPreservesWrappedStringFrames()
{
    CircularStringBuff buffer(8);
    char output[8] = {};
    size_t length = 0;

    CHECK(!buffer.putString("ab"));
    CHECK(!buffer.putString("cd"));
    CHECK(!buffer.getString(output, sizeof(output), &length));
    CHECK(length == 2 && strcmp(output, "ab") == 0);

    CHECK(!buffer.putString("efg"));
    CHECK(!buffer.getString(output, sizeof(output), &length));
    CHECK(length == 2 && strcmp(output, "cd") == 0);
    CHECK(!buffer.getString(output, sizeof(output), &length));
    CHECK(length == 3 && strcmp(output, "efg") == 0);
    CHECK(buffer.empty());
}

void testReceiverFrequencyBoundsAreInclusive()
{
    CHECK(receiverFrequencyIsSupported(RECEIVER_FREQUENCY_MIN_HZ));
    CHECK(receiverFrequencyIsSupported(RECEIVER_FREQUENCY_MAX_HZ));
    CHECK(!receiverFrequencyIsSupported(RECEIVER_FREQUENCY_MIN_HZ - 1));
    CHECK(!receiverFrequencyIsSupported(RECEIVER_FREQUENCY_MAX_HZ + 1));
}

void testReceiverFrequencyStepRejectsOutOfBandResult()
{
    uint32_t result = RECEIVER_FREQUENCY_MIN_HZ;

    CHECK(!receiverFrequencyStep(RECEIVER_FREQUENCY_MIN_HZ, -1, 100, &result));
    CHECK(result == RECEIVER_FREQUENCY_MIN_HZ);

    result = RECEIVER_FREQUENCY_MAX_HZ;
    CHECK(!receiverFrequencyStep(RECEIVER_FREQUENCY_MAX_HZ, 1, 1000, &result));
    CHECK(result == RECEIVER_FREQUENCY_MAX_HZ);
}

void testReceiverFrequencyStepProducesValidatedResult()
{
    uint32_t result = 0;

    CHECK(receiverFrequencyStep(3570500, 1, 1000, &result));
    CHECK(result == 3571500);
    CHECK(receiverFrequencyStep(result, -1, 100, &result));
    CHECK(result == 3571400);
    CHECK(!receiverFrequencyStep(result, 1, 0, &result));
    CHECK(!receiverFrequencyStep(result, 2, 100, &result));
    CHECK(!receiverFrequencyStep(result, 1, 100, 0));
}

} // namespace

int main()
{
    testCircularBufferExactFitIncludesTerminator();
    testCircularBufferRejectsOversizeWithoutMutation();
    testCircularBufferRejectsUnframedDataWithoutMutation();
    testCircularBufferRejectsSmallDestinationWithoutMutation();
    testCircularBufferPopReturnsMostRecentByte();
    testCircularBufferPreservesWrappedStringFrames();
    testReceiverFrequencyBoundsAreInclusive();
    testReceiverFrequencyStepRejectsOutOfBandResult();
    testReceiverFrequencyStepProducesValidatedResult();

    if (failures != 0) {
        fprintf(stderr, "%d regression test(s) failed\n", failures);
        return 1;
    }

    puts("All host regression tests passed");
    return 0;
}
