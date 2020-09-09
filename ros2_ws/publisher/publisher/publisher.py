#!/usr/bin/env python3

import os
import sys
import time
import struct
import select
import ctypes
import threading
import multiprocessing as mp

import serial
from termcolor import cprint

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

class ReadProc(mp.Process):

    def __init__(self, port, baud, stop_event, data_q):
        super().__init__(name='read')

        self.port = port
        self.baud = baud
        self.data_q = data_q
        self._stop_event = stop_event
    
    def join(self, timeout=1):
        self._stop_event.set()
        return super().join(timeout)

    def open_dev(self):
        try:
            self.dev = serial.Serial(self.port, self.baud, timeout=0.01)
            self.dev.flush()
            self.dev.flushInput()
            return True
        except Exception as ex:
            cprint('failed to open data port', 'red', 'on_white', attrs=['bold'])
            return False

    def run(self):
        cprint('reader pid: %d' %os.getpid())
        self.open_dev()

        while not self._stop_event.is_set():

            try:
                r,_,_ = select.select([self.dev], [], [], 0.1)
            except:
                self._stop_event.set()
                break

            if not r:
                continue
            else:
                dev = r[0]
                data = dev.read(dev.in_waiting)
                self.data_q.put(data)
        
        cprint('read proc end.', 'green')   

class ParseProc(mp.Process):

    MAGIC = b'\x02\x01\x04\x03\x06\x05\x08\x07'
    HEAD_PATTERN = 'I'*8

    TAG_DICT = {1:('MSG_DETECTED_POINTS', 'f'*4), 2:('MSG_RANGE_PROF', 'H'), 3:('MSG_NIOSE_PROF', 'H'), 4:('MSG_AZIMUT_STATIC_HEAT_MAP', 'HH'),
    5:('MSG_RANGE_DOPPLER_HEAT_MAP', 'H'), 6:('MSG_STATS', 'I'*6), 7:('MSG_DETECTED_POINTS_SIDE_INFO', 'HH'), 8:('MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP', 'HH'), 9:('MSG_TEMP', 'II'+'H'*10)}


    def __init__(self, stop_event, data_q):
        super().__init__(name='parse')

        self._stop_event = stop_event
        self.data_q = data_q

        self.raw_data = b''

        self.lock = threading.RLock()
    
    def join(self, timeout=1):
        self._stop_event.set()
        return super().join(timeout)

    def padding_thread(self):
        while not self.padding_thread_stop_ev.is_set():
            try:
                data = self.data_q.get()

                with self.lock:
                    self.raw_data += data
                    # cprint(len(self.raw_data), 'blue', attrs=['dark'])
                    # cprint('-> %s' %self.raw_data)
            except:
                break
        
        cprint('pading thread end.', 'green')
        return

    def run(self):
        # rclpy.init()
        self.ros_node = rclpy.create_node('mmwave_publisher')
        self.ros_pub = self.ros_node.create_publisher(PointCloud2, 'cloud_in', 10)

        cprint('parser pid: %d' %os.getpid())
        head_idx = -1
        next_head_idx = -1
        under_pack = False

        self.padding_thread_stop_ev = threading.Event()
        padding_thread = threading.Thread(target=self.padding_thread)
        padding_thread.start()

        while not self._stop_event.is_set():

            head_idx = self.raw_data.find(self.MAGIC)
            if head_idx == -1:
                # cprint('not found head', 'yellow')
                # time.sleep(0.01)
                continue

            next_head_idx = self.raw_data.find(self.MAGIC, head_idx+8)
            if next_head_idx == -1:
                # cprint('not found next head', 'yellow')
                try:
                    time.sleep(0.001)
                except:
                    break
                continue
            
            data = self.raw_data[head_idx+8: next_head_idx]
            with self.lock:
                self.raw_data = self.raw_data[next_head_idx:]
            self.parse_frame(data)
    
        self.ros_node.destroy_node()
        cprint('parse proc end.', 'green')
        self.padding_thread_stop_ev.set()
        padding_thread.join(.1)
        time.sleep(.1)
    
    def parse_frame(self, data):
        # cprint(str(data), 'yellow', attrs=['dark'])
        msg = PointCloud2()
        msg.header.frame_id = 'base_radar_link'

        try:                                    
            h_version, h_total_len, h_platform, h_frame_no, h_time_cpu_cycle, h_detect_obj_no, h_tlvs_no, h_sub_frame_no = struct.unpack('I'*8, data[:32])
        except:
            cprint('frame header error', 'white', 'on_yellow')
            return

        if len(data) != h_total_len-8:
            cprint('frame len error', 'white', 'on_red')
            return

        msg.height = 1
        msg.width = h_detect_obj_no
        msg.is_dense = True;
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='v', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='snr', offset=16, datatype=PointField.INT16, count=1),
            PointField(name='noise', offset=18, datatype=PointField.INT16, count=1)
        ]
        msg.point_step = 20
        msg.row_step = 20 * h_detect_obj_no

        cprint('0x%x, %d, 0x%x, %d, %d, %d, %d, %d' %(h_version, h_total_len, h_platform, h_frame_no, h_time_cpu_cycle, h_detect_obj_no, h_tlvs_no, h_sub_frame_no), 'yellow', 'on_blue', attrs=['bold'])
        # cprint(str(data), 'yellow')

        tlvs_tag_idx = 32

        tlvs_val_d = {}

        for i in range(h_tlvs_no):

            tlvs_tag, tlvs_len = struct.unpack('II', data[tlvs_tag_idx:tlvs_tag_idx+8])
            tlvs_val = data[tlvs_tag_idx+8 : tlvs_tag_idx+8+tlvs_len]
            tlvs_tag_idx += (8+tlvs_len)
            
            # cprint('\ttlvs_tag: %d, len: %d' %(tlvs_tag, tlvs_len), 'red', 'on_white')

            if tlvs_tag == 1:

                pattern = self.TAG_DICT[tlvs_tag][1]*h_detect_obj_no
                tlvs_val = list(struct.unpack(pattern, tlvs_val))
                idx = 0
                for i in range(h_detect_obj_no):
                    tlvs_val[idx], tlvs_val[idx+1] = tlvs_val[idx+1], -tlvs_val[idx]
                    idx += 4
                
                tlvs_val_d[tlvs_tag] = tlvs_val
            
            elif tlvs_tag == 7:
                pattern = self.TAG_DICT[tlvs_tag][1]*h_detect_obj_no
                tlvs_val = list(struct.unpack(pattern, tlvs_val))

                tlvs_val_d[tlvs_tag] = tlvs_val
        
        pos_info = tlvs_val_d.get(1, [])
        pos_info = [pos_info[i:i+4] for i in range(0, len(pos_info), 4)]

        side_info = tlvs_val_d.get(7, [])
        side_info = [side_info[i:i+2] for i in range(0, len(side_info), 2)]

        # cprint(str(pos_info))
        # cprint(str(side_info))

        data = []
        passed = 0
        for i in range(h_detect_obj_no):
            if side_info[i][0] < 100:
                passed += 1
                cprint('low snr point, pass')
            else:
                data += pos_info[i] + side_info[i]
        
        pattern = (self.TAG_DICT[1][1] + self.TAG_DICT[7][1]) * (h_detect_obj_no - passed)

        msg.width = h_detect_obj_no - passed
        msg.row_step = 20 * (h_detect_obj_no - passed)
        msg.data = struct.pack(pattern, *data)
        
        # cprint(str(data))

        self.ros_pub.publish(msg)



class Paser( object ):

    CONF_PORT = '/dev/serial/by-id/usb-Texas_Instruments_XDS110__03.00.00.05__Embed_with_CMSIS-DAP_DB041204-if00'
    CONF_BAUD = 115200
    CONF_FILE = 'conf/current.cfg'
    DATA_PORT = '/dev/serial/by-id/usb-Texas_Instruments_XDS110__03.00.00.05__Embed_with_CMSIS-DAP_DB041204-if03'
    DATA_BAUD = 921600

    def __init__(self):
        pass

    def conf_dev(self):

        try:
            dev = serial.Serial(self.CONF_PORT, self.CONF_BAUD, timeout=0.01)
        except Exception as ex:
            print(ex)
            return False
        
        try:
            conf_lines = open(self.CONF_FILE).readlines()
        except Exception as ex:
            print(ex)
            return False
        
        for l in conf_lines:
            if l.startswith('%'):
                continue
            
            try:
                dev.write(l.encode('latin-1'))
                print(dev.readlines())
            except Exception as ex:
                print(ex)
                return False
        dev.close()
        
        return True
    
    def loop(self):
        # self.shm_raw = mp.Value(ctypes.c_char_p, b'', lock=True)
        cprint('main pid: %d' %os.getpid())
        self.data_q = mp.Queue()
    
        self._stop_event = mp.Event()
        
        self._launch_read_proc()
        self._launch_parse_proc()

        while 1:
            try:
                time.sleep(10)
                # cprint('%s, %s' %(self.read_proc.is_alive(), self.parse_proc.is_alive()))
            except:
                break
        
        self._stop_event.set()
        self.read_proc.join()
        self.parse_proc.join()

        cprint('bye', 'white', 'on_blue')
        time.sleep(1)

    
    def _launch_read_proc(self):

        if self.conf_dev():
            print('dev conf done')
        else:
            print('dev conf failed..')
            sys.exit(-1)

        self.read_proc = ReadProc(self.DATA_PORT, self.DATA_BAUD, self._stop_event, self.data_q)
        self.read_proc.start()

    def _launch_parse_proc(self):
        self.parse_proc = ParseProc(self._stop_event, self.data_q)
        self.parse_proc.start()


######################
def main():
    rclpy.init()
    p = Paser()
    p.loop()
    rclpy.shutdown()


if __name__ == "__main__":

    main()