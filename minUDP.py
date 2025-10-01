print("=================")
print("MinUDP")
print("=================")

from mavlink.pymavlink import mavutil
import os
os.environ['MAVLINK20'] = '1'
import time
import argparse
import sys
import socket
import subprocess
import shlex
import threading

def hb_thread_func(udpcon,timeout=2):
    while (True):
        udpcon.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

        udpcon.mav.ping_send(
            0,      #timestamp
            1,      #sequence number
            126,    #SITL MAVLINK sys ID
            0)
        print('Sent HB')
        time.sleep(timeout)

def main():
    parser = argparse.ArgumentParser(description="MinUDP - Hololens2 companion streamer",
                                     epilog="kieran.wood@manchester.ac.uk")
    parser.add_argument('-s', '--sim', action='store_true', help='run for SITL')
    parser.add_argument('-r', '--real', action='store_true', help='run for real')
    parser.add_argument('-v', '--verbose',    action='store_true', help=':verbose output')
    args = parser.parse_args()

    mav_ip = '127.0.0.1'
    mav_port = 16000

    real_ip = '192.168.1.160'
    real_ports = [14240]

    udp_ip_pc = '127.0.0.1'
    udp_ip_hl = '192.168.0.34'
    udp_port = 1010

    bootTime = time.time()
    print('Starting....')


    if args.sim:
        routing_string = "mavp2p --hb-systemid=124 --streamreq-disable tcpc:127.0.0.1:14555 tcps:127.0.0.1:16000 tcps:127.0.0.1:16001"
        subprocess.Popen(shlex.split(routing_string),
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.STDOUT)
        print('...sim only')

    if args.real:
        tmpstring = ''
        for port in real_ports:
            tmpstring +='udp:'
            tmpstring += real_ip
            tmpstring += (':%d ' % port)
        routing_string = "mavp2p --hb-systemid=124 --streamreq-disable tcpc:127.0.0.1:14555 %s tcps:127.0.0.1:16000 tcps:127.0.0.1:16001" % tmpstring
        print(routing_string)
        subprocess.Popen(shlex.split(routing_string),
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.STDOUT)
        print('...sim and real')

    connection_string = 'tcp:%s:%d' % (mav_ip,mav_port)
    MVLK = mavutil.mavlink_connection(connection_string,source_system=254,dialect='ardupilotmega')

    sock_pc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_hl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    hb_thread = threading.Thread(target=hb_thread_func, name='HB thread', args=(MVLK,5,),daemon=True)
    hb_thread.start()

    while(True):
        msg_count = -1
        while(True):
            msg = None
            msg = MVLK.recv_match(blocking=False)
            msgtime = time.time()
            msg_count = msg_count + 1

            if not msg:
                break
            elif msg.get_type() == "BAD_DATA":
                if mavutil.all_printable(msg.data):
                    sys.stdout.write(msg.data)
                    sys.stdout.flush()
                    print('Bad MAVLink message received')
            else:
                sysid  = msg.get_srcSystem()
                if msg.get_type() == "GLOBAL_POSITION_INT":
                    #msg #33 uses an int32 in millimeters for altitude, int32 in degE7 for lon and lat
                    print("GLOBAL_POSITION_INT:SYSID = %s, lat=%s, lon=%s, alt=%s, gpsalt=%s" % (sysid,msg.lat/1e7,msg.lon/1e7,msg.relative_alt/1e3,msg.alt/1e3))
                    lat = msg.lat/1e7
                    lon = msg.lon/1e7
                    alt = msg.alt/1e3
                    bat = 100.0

                    udp_msg = ''
                    udp_msg += '%d,' % sysid
                    udp_msg += '%f,' % lat
                    udp_msg += '%f,' % lon
                    udp_msg += '%f,' % alt
                    udp_msg += '%f' % bat
            
                    sock_pc.sendto(udp_msg.encode('utf-8'), (udp_ip_pc, udp_port))
                    sock_hl.sendto(udp_msg.encode('utf-8'), (udp_ip_hl, udp_port))
                    print('Sent GPS UDP: %s' % udp_msg)
        
        time.sleep(0.01)

if __name__ == "__main__":
    main()


print("=================")
print('Done')
print("=================")


#eof