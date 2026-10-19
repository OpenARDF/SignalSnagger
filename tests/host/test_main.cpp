#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "CircularStringBuff.h"
#include "datetime_utils.h"
#include "periodic_countdown.h"
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

void testDateTimeParserAcceptsSupportedFormats()
{
    struct tm parsed = {};

    CHECK(!mystrptime("2000-02-29T23:59:58Z", &parsed));
    CHECK(parsed.tm_year == 100);
    CHECK(parsed.tm_mon == 1);
    CHECK(parsed.tm_mday == 29);
    CHECK(parsed.tm_yday == 59);
    CHECK(parsed.tm_hour == 23);
    CHECK(parsed.tm_min == 59);
    CHECK(parsed.tm_sec == 58);

    CHECK(!mystrptime("2026-10-19T08:05", &parsed));
    CHECK(parsed.tm_year == 126);
    CHECK(parsed.tm_mon == 9);
    CHECK(parsed.tm_mday == 19);
    CHECK(parsed.tm_yday == 291);
    CHECK(parsed.tm_hour == 8);
    CHECK(parsed.tm_min == 5);
    CHECK(parsed.tm_sec == 0);

    CHECK(!mystrptime("2026-10-19T08:05Z", &parsed));
    CHECK(parsed.tm_sec == 0);

    CHECK(!mystrptime("2026-10-19T08:05:06", &parsed));
    CHECK(parsed.tm_sec == 6);
}

void testDateTimeParserRejectsMalformedDatesWithoutMutation()
{
    static const char* invalid[] = {
        "2025-02-29T12:00:00Z",
        "2100-02-29T12:00:00Z",
        "2024-04-31T12:00:00Z",
        "2024/04/30T12:00:00Z",
        "2024-04-30 12:00:00Z",
        "2024-04-30T12:00:00extra",
        "2024-04-30T12:00:",
        ""
    };

    for(size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        struct tm parsed;
        struct tm original;
        memset(&parsed, 0x5a, sizeof(parsed));
        original = parsed;
        CHECK(mystrptime(invalid[i], &parsed));
        CHECK(memcmp(&parsed, &original, sizeof(parsed)) == 0);
    }

    struct tm parsed = {};
    CHECK(mystrptime(0, &parsed));
    CHECK(mystrptime("2024-04-30T12:00:00Z", 0));
}

void testDateTimeEpochConversion()
{
    CHECK(convertTimeStringToEpoch("2000-01-01T00:00:00Z") == UINT32_C(946684800));
    CHECK(convertTimeStringToEpoch("2024-02-29T00:00:00Z") == UINT32_C(1709164800));
    CHECK(convertTimeStringToEpoch("invalid") == 0);
}

void testPeriodicCountdownDoesNotDriftAtByteWrap()
{
    uint8_t ticksRemaining = 6;
    unsigned elapsedCount = 0;

    for(unsigned tick = 1; tick <= 600; ++tick)
    {
        bool elapsed = periodicCountdownTick(&ticksRemaining, 6);
        CHECK(elapsed == (tick % 6 == 0));
        if(elapsed)
        {
            ++elapsedCount;
        }
    }

    CHECK(elapsedCount == 100);
    CHECK(ticksRemaining == 6);
}

void testPeriodicCountdownRejectsInvalidArguments()
{
    uint8_t ticksRemaining = 4;
    CHECK(!periodicCountdownTick(0, 6));
    CHECK(!periodicCountdownTick(&ticksRemaining, 0));
    CHECK(ticksRemaining == 4);

    ticksRemaining = 0;
    CHECK(periodicCountdownTick(&ticksRemaining, 6));
    CHECK(ticksRemaining == 6);
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
    testDateTimeParserAcceptsSupportedFormats();
    testDateTimeParserRejectsMalformedDatesWithoutMutation();
    testDateTimeEpochConversion();
    testPeriodicCountdownDoesNotDriftAtByteWrap();
    testPeriodicCountdownRejectsInvalidArguments();

    if (failures != 0) {
        fprintf(stderr, "%d regression test(s) failed\n", failures);
        return 1;
    }

    puts("All host regression tests passed");
    return 0;
}
