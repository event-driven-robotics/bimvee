import numpy as np


# Types of events outputed by the ATIS3
CD_OFF = 0
CD_ON = 1
EVT_TIME_HIGH = 8
EXT_TRIGGER = 12
OTHERS = 14
CONTINUED = 15


def importRawHeaders(raw):

    # Go to the beginning of the file
    raw.seek(0)
    info = {}
    # Read the first character
    is_comment = '%' in str(raw.read(1))
    while is_comment:
        # Read the rest of the line
        line = raw.readline().decode('utf-8', "ignore")
        # Pick out date and time of recording
        #  % Date YYY-MM-DDHH:MM:SS
        if line[: 6] == ' Date ':
            info['dateTime'] = line[6:-1]
        # Integrator name
        if line[: 17] == ' integrator_name ':
            info['IntegratorName'] = line[17:-1]
        # Plugin name
        if line[: 13] == ' plugin_name ':
            info['PluginName'] = line[13:-1]
        # Serial Number
        if line[: 15] == ' serial_number ':
            info['SerialNumber'] = line[15:-1]
        # EVT
        if line[: 5] == ' evt ':
            info['evt'] = line[5:-1]
        # Firmware Version
        if line[: 18] == ' firmware_version ':
            info['FirmwareVersion'] = line[18:-1]
        # System ID
        if line[: 11] == ' system_ID ':
            info['SystemID'] = line[11:-1]
        # Subsystem ID
        if line[: 14] == ' subsystem_ID ':
            info['SubsystemID'] = line[14:-1]
        # Horizontal size of image sensor array
        if line[: 7] == ' Width ':
            info['Width'] = line[7:-1]
        # Horizontal size of image sensor array
        if line[: 8] == ' Height ':
            info['Height'] = line[8:-1]
        # Read ahead the first character of the next line to complete the
        # while loop
        is_comment = '%' in str(raw.read(1))
    # We have read ahead one byte looking for '#', and not found it.
    # Now wind back one to be in the right place to start reading
    raw.seek(-1, 1)

    return info


class TimestampedList(list):
    def __init__(self, ts_correspondences, ts_associated_data):
        self.ts_correspondences = ts_correspondences
        self.ts_associated_data = ts_associated_data

    def __getitem__(self, i):
        if type(i) == slice:
            return [self[x] for x in range(i.start, i.stop)]
        elif hasattr(i, '__len__'):
            return [self[x] for x in i]
        else:
            return self.ts_associated_data[self.ts_correspondences[i]:self.ts_correspondences[i + 1]]

    def __len__(self):
        return len(self.ts_correspondences)


def extract_events_from_data_file(file_stream):

    file_stream.seek(0)
    info = importRawHeaders(file_stream)
    timestamps = []
    bitstrings = []
    ts_correspondences = []
    offset = 0
    while True:
        data = file_stream.read(40000)
        if not data:
            break
        events = np.frombuffer(data, '<u4')
        ev_type = (events & 0xF0000000) >> 28
        is_timestamp_msb = ev_type == EVT_TIME_HIGH

        timestamps.append(events[is_timestamp_msb])
        ts_correspondences.append(np.where(is_timestamp_msb)[0] + offset - np.arange(len(timestamps[-1])))
        bitstrings.append(events[~is_timestamp_msb])
        offset += len(bitstrings[-1])

    timestamps = np.concatenate(timestamps) / 1000000
    bitstrings = np.concatenate(bitstrings)
    ts_correspondences = np.concatenate(ts_correspondences)
    ts_map = TimestampedList(ts_correspondences, bitstrings)
    return timestamps, ts_map


def decode_events(bitstrings_array):
    ev_type = (bitstrings_array & 0xF0000000) >> 28
    notEvents = ev_type > 1
    bitstrings_array = bitstrings_array[~notEvents]  # Filtering non events
    y = (bitstrings_array & 0x000007FF).astype(int)           # y is bits 10..0
    x = ((bitstrings_array & 0x003FF800) >> 11).astype(int)    # x is bits 21..11
    pol = ev_type[~notEvents] == 1
    # if timestamps is not None:
    #     ts_LSB = ((bitstrings_array & 0x0FC00000) >> 22) / 1000000  # ts is bits 27..22
    #     ts = (timestamps + ts_LSB)
    #     return pol, x, y, ts
    return pol, x, y
