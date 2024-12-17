import re
import numpy as np


def unquoting(match):
    matchedString = (match.string[match.span()[0]:match.span()[1]])
    even = len(matchedString) % 2 == 0
    if matchedString[-2:] == b'\\0':
        return b'\\' * ((len(matchedString) - 1) // 2) + b'\0' if even\
            else b'\\' * ((len(matchedString) - 1) // 2) + b'0'
    if matchedString[-2:] == b'\\n':
        return b'\\' * ((len(matchedString) - 1) // 2) + b'\n' if even\
            else b'\\' * ((len(matchedString) - 1) // 2) + b'n'
    if matchedString[-2:] == b'\\r':
        return b'\\' * ((len(matchedString) - 1) // 2) + b'\r' if even\
            else b'\\' * ((len(matchedString) - 1) // 2) + b'r'
    if matchedString[-2:] == b'\\"':
        return b'\"'
    if matchedString[-2:] == b'\\\\':
        if match.string[match.span()[1]:match.span()[1]+1] in [b'0', b'n', b'r', b'\0', b'\n', b'\r']:
            return matchedString
        return b'\\' * ((len(matchedString)) // 2)


def fromStringNested(in_string):
    return re.sub(b'\\\{2,}|\\\\\"', unquoting, re.sub(b'\\\\+[nr0]', unquoting, in_string))


def extract_events_from_data_file(data_file):
    eventsToDecode = []
    timestamps = []
    check_if_with_ts = False
    with_ts = False
    for c in data_file:
        firstQuoteIdx = c.find(b'\"')
        lastQuoteIdx = c[::-1].find(b'\"')
        bottleNum, ts, bottleType, _ = c[:firstQuoteIdx - 1].decode().split(' ')
        data = c[firstQuoteIdx + 1:-(lastQuoteIdx + 1)]
        bitStrings = np.frombuffer(fromStringNested(data), np.uint32)
        if not check_if_with_ts and len(bitStrings) > 10:
            with_ts = np.all(sorted(bitStrings[::2]) == bitStrings[::2])
            check_if_with_ts = True
            timestamps.append(float(ts))
        if check_if_with_ts:
            if not with_ts:
                timestamps.append(float(ts))
            else:
                timestamps.clear()

        eventsToDecode.append(bitStrings)

    if with_ts:
        timestamps, eventsToDecode = np.reshape(np.concatenate(eventsToDecode), (-1, 2))

    return np.array(timestamps), eventsToDecode


def decode_events(bitstrings):
    pol = ~np.array(bitstrings & 0x01, dtype=bool)  # We want True=ON=brighter, False=OFF=darker, so we negate
    x = np.uint16(bitstrings >> 1 & 0x7FF)
    y = np.uint16(bitstrings >> 12 & 0x3FF)
    ch = np.uint8(bitstrings >> 23 & 0x01) #TODO check if channel is useful
    return pol, x, y
