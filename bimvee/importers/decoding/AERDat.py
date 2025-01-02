import struct
import numpy as np
from ctypes import c_uint32

def extract_events_from_data_file(file_stream):
    packet_format = 'BHHI'  # pol = uchar, (x,y) = ushort, t = uint32
    packet_size = struct.calcsize('=' + packet_format)  # 16 + 16 + 8 + 32 bits => 2 + 2 + 1 + 4 bytes => 9 bytes
   
    content = np.array(bytearray(file_stream.read()))
    extra_bits = len(content) % packet_size

    '''Remove Extra Bits'''
    if extra_bits:
        content = content[0:-extra_bits]
    
    reshaped_content = content.reshape((-1,9))
    
    timestamps = np.frombuffer(reshaped_content[:, -4:].tobytes(), dtype=np.uint32)
    bitstrings = reshaped_content[:, :5]

    return timestamps / 1e6, bitstrings



def decode_events(bitstrings, timestamps):
    assert(len(bitstrings) == len(timestamps))
    unpacked_data = np.array(struct.unpack('=' + 'BHH' * len(bitstrings), b''.join([x.tobytes() for x in bitstrings])))
    pol = unpacked_data[::3]
    y = unpacked_data[1::3]
    x = unpacked_data[2::3]
    return pol, x, y, timestamps
