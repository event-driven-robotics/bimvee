import struct
import numpy as np

def importAerdat(**kwargs):
    filePathOrName = kwargs['filePathOrName']
    with open(filePathOrName, 'rb') as f:
        content = f.read()

    ''' Packet format'''
    packet_format = 'BHHI'  # pol = uchar, (x,y) = ushort, t = uint32
    packet_size = struct.calcsize('=' + packet_format)  # 16 + 16 + 8 + 32 bits => 2 + 2 + 1 + 4 bytes => 9 bytes
    num_events = len(content) // packet_size
    extra_bits = len(content) % packet_size

    '''Remove Extra Bits'''
    if extra_bits:
        content = content[0:-extra_bits]

    ''' Unpacking'''
    event_list = list(struct.unpack('=' + packet_format * num_events, content))

    timestamps = np.array(event_list[3:][::4], dtype=float)
    x = np.array(event_list[2:][::4])
    y = np.array(event_list[1:][::4])
    pol = np.array(event_list[::4])
    tsOffset = timestamps[0]
    timestamps -= tsOffset
    timestamps /= 1e6
    out_dict = {'info': {'tsOffset': tsOffset,
                         'filePathOrName': filePathOrName,
                         'fileFormat': 'aerdat'},
                'data': {
                    'ch0': {
                        'dvs': {
                            'ts': timestamps,
                            'x': x,
                            'y': y,
                            'pol': pol
                        }}}}
    return out_dict
