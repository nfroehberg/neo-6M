import struct
import serial
import sys
import re
if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
    from collections import OrderedDict

# dictionary of command references for ublox neo 6m UBX commands
# struct encoding match with ublox data types:
# X1=B, X2=H, X4=I, U1=B, U2=H, U4=I, I1=b, I2=h, I4=i, R4=f, R8=d, CH=c
cmd_ref = {
    'ACK': {'Cls': 0x05, 'command': {
        'ACK': {'ID': 0x01, 'length': 2, 'encoding': 'BB', 'payload': ['clsID','msgID']},
        'NAK': {'ID': 0x00, 'length': 2, 'encoding': 'BB', 'payload': ['clsID','msgID']}
        }},
    'AID': {'Cls': 0x0b, 'Command': {
        'ALM': {'ID': 0x30},
        'ALPSRV': {'ID': 0x32},
        'ALP': {'ID': 0x50},
        'AOP': {'ID': 0x33},
        'DATA': {'ID': 0x10},
        'EPH': {'ID': 0x31},
        'HUI': {'ID': 0x02},
        'INI': {'ID': 0x01},
        'REQ': {'ID': 0x00}
        }},
    'CFG': {'Cls': 0x06, 'command': {
        # still need to implement bitfield encoding, for now encoded as shorts/ints
        'ANT': {'ID': 0x13, 'length': 4, 'encoding': 'HH', 'payload': ['flags','pins']},
        # CFG also has optional deviceMask parameter, on firmware 7.03 replaced ny NVS
        'CFG': {'ID': 0x13, 'length': 12, 'encoding': 'III', 'payload': ['clearMask','saveMask','loadMask']},
        'DAT': {'ID': 0x06},
        'EKF': {'ID': 0x12},
        'ESFGWT': {'ID': 0x29},
        'FXN': {'ID': 0x0e},
        'INF': {'ID': 0x02},
        'ITFM': {'ID': 0x39},
        'MSG': {'ID': 0x01},
        'NAV5': {'ID': 0x24, 'length': 36, 'encoding': 'HBBiIbBHHHHBBIII', 'payload': ['mask','dynModel','fixMode',
                                                                                         'fixedAlt','fixedAltVar','minElev',
                                                                                         'drLimit','pDop','tDop','pAcc',
                                                                                         'tAcc','staticHoldThresh','dgpsTimeOut',
                                                                                         'reserved2','reserved3','reserved4'
                                                                                         ]},
        'NAVX5': {'ID': 0x23, 'length': 40, 'encoding': 'HHIBBBBBBBBBBHIBBBBBBHII', 'payload': ['version','mask1','reserved0'
                                                                                                'reserved1','reserved2','minSVs',
                                                                                                'maxSVs','minCNO','reserved5',
                                                                                                'iniFix3D','reserved6','reserved7',
                                                                                                'reserved8','wknRollover','reserved9',
                                                                                                'reserved10','reserved11','usePPP',
                                                                                                'useAOP','reserved12','reserved13',
                                                                                                'aopOrbMaxErr','reserved3','reserved4'
                                                                                                ]},
        'NMEA': {'ID': 0x17, 'length': 4, 'encoding': 'BBBB', 'payload': ['filter','version','numSV','flags']},
        'NVS': {'ID': 0x22, 'length': 13, 'encoding': 'IIIB', 'payload': ['clearMask','saveMask','loadMask','deviceMask']},
        'PM2': {'ID': 0x3b, 'length': 44, 'encoding': 'BBBBIIIIHHHHIIBBHI', 'payload': ['version','reserved1','reserved2','reserved3',
                                                                                          'flags','updatePeriod','searchPeriod','gridOffset',
                                                                                          'onTime','minAcqTime','reserved4','reserved5',
                                                                                          'reserved6','reserved7','reserved8','reserved9',
                                                                                          'reserved10','reserved11'
                                                                                          ]},
        'PM': {'ID': 0x32, 'length': 24, 'encoding': 'BBBBIIIIHH', 'payload': ['version','reserved1','reserved2','reserved3',
                                                                                 'flags','updatePeriod','searchPeriod','gridOffset',
                                                                                 'onTime','minAcqTime'
                                                                                 ]},
        'PRT': {'ID': 0x00},
        'RATE': {'ID': 0x08, 'length': 6, 'encoding': 'HHH', 'payload': ['measRate','navRate','timeRef']},
        'RINV': {'ID': 0x34},
        'RST': {'ID': 0x04, 'length': 4, 'encoding': 'HBB', 'payload': ['navBbrMask','resetMode','reserved1']},
        'RXM': {'ID': 0x11, 'length': 2, 'encoding': 'BB', 'payload': ['reserved1','lpMode']},
        'SBAS': {'ID': 0x16},
        'TMODE2': {'ID': 0x3d},
        'TMODE': {'ID': 0x1d},
        'TP5': {'ID': 0x31},
        'TP': {'ID': 0x07},
        'USB': {'ID': 0x1b}
        }},
    'ESF': {'Cls': 0x10, 'command': {
        'MEAS': {'ID': 0x02},
        'STATUS': {'ID': 0x10}
        }},
    'INF': {'Cls': 0x04, 'command': {
        'DEBUG': {'ID': 0x04},
        'ERROR': {'ID': 0x00},
        'NOTICE': {'ID': 0x02},
        'TEST': {'ID': 0x03},
        'WARNING': {'ID': 0x01}
        }},
    'MON': {'Cls': 0x0a, 'command': {
        'HW2': {'ID': 0x0b},
        'HW': {'ID': 0x09},
        'IO': {'ID': 0x02},
        'MSGPP': {'ID': 0x06},
        'RXBUF': {'ID': 0x07},
        'RXR': {'ID': 0x21},
        'TXBUF': {'ID': 0x08},
        'VER': {'ID': 0x04}
        }},
    'NAV': {'Cls': 0x01, 'command': {
        'AOPSTATUS': {'ID': 0x60},
        'CLOCK': {'ID': 0x22},
        'DGPS': {'ID': 0x31},
        'DOP': {'ID': 0x04},
        'EKFSTATUS': {'ID': 0x40},
        'POSECEF': {'ID': 0x01},
        'POSLLH': {'ID': 0x02},
        'SBAS': {'ID': 0x32},
        'SOL': {'ID': 0x06},
        'STATUS': {'ID': 0x03},
        'SVINFO': {'ID': 0x30},
        'TIMEGPS': {'ID': 0x20},
        'TIMEUTC': {'ID': 0x21},
        'VELECEF': {'ID': 0x11},
        'VELNED': {'ID': 0x12}
        }},
    'RXM': {'Cls': 0x02, 'command': {
        'ALM': {'ID': 0x30},
        'ALM': {'ID': 0x31},
        'PMREQ': {'ID': 0x41, 'length': 8, 'encoding': 'II', 'payload': ['duration','flags']},
        'RAW': {'ID': 0x10},
        'SFRB': {'ID': 0x11},
        'SVSI': {'ID': 0x20}
        }},
    'TIM': {'Cls': 0x0d, 'command': {
        'SVIN': {'ID': 0x04},
        'TM2': {'ID': 0x03},
        'TP': {'ID': 0x01},
        'VRFY': {'ID': 0x06}
        }},
    }

# class and id reference for decoding ubx messages
cls_ref = {0x05:'ACK',0x0b:'AID',0x06:'CFG',0x10:'ESF',0x04:'INF',0x0a:'MON',0x01:'NAV',0x02:'RXM',0x0d:'TIM'}

id_ref = {
    'ACK':{0x01:'ACK',0x00:'NAK'},
    'AID':{0x30:'ALM',0x32:'ALPSRV',0x50:'ALP',0x33:'AOP',0x10:'DATA',0x31:'EPH',0x02:'HUI',0x01:'INI',0x00:'REQ'},
    'CFG':{0x13:'ANT',0x09:'CFG',0x06:'DAT',0x12:'EKF',0x29:'ESFGWT',0x0e:'FXN',0x02:'INF',0x39:'ITFM',0x01:'MSG',
           0x24:'NAV5',0x23:'NAVX5',0x17:'NMEA',0x22:'NVS',0x3b:'PM2',0x32:'PM',0x00:'PRT',0x08:'RATE',0x34:'RINV',
           0x04:'RST',0x11:'RXM',0x16:'SBAS',0x3d:'TMODE2',0x1d:'TMODE',0x31:'TP5',0x07:'TP',0x1b:'USB'},
    'ESF':{0x02:'MEAS',0x10:'STATUS'},
    'INF':{0x04:'DEBUG',0x00:'ERROR',0x02:'NOTICE',0x03:'TEST',0x01:'WARNING'},
    'MON':{0x0b:'HW2',0x09:'HW',0x02:'IO',0x06:'MSGPP',0x07:'RXBUF',0x21:'RXR',0x08:'TXBUF',0x04:'VER'},
    'NAV':{0x60:'OPSTATUS',0x22:'CLOCK',0x31:'DGPS',0x04:'DOP',0x40:'EKFSTATUS',0x01:'POSECEF',0x02:'POSLLH',
           0x32:'SBAS',0x06:'SOL',0x03:'STATUS',0x30:'SVINFO',0x20:'TIMEGPS',0x21:'TIMEUTC',0x11:'VELECEF',0x12:'VELNED'},
    'RXM':{0x30:'ALM',0x31:'EPH',0x41:'PMREQ',0x10:'RAW',0x11:'SFRB',0x20:'SVSI'},
    'TIM':{0x04:'SVIN',0x03:'TM2',0x01:'TP',0x06:'VRFY'}
    }


SYNC_CHAR_1 = 0xb5
SYNC_CHAR_2 = 0x62

class Neo6M_UBX():

    def __init__(self, port, b_rate = 9600):
        
        self.port = serial.Serial(port,b_rate)
        self.b_rate = 9600

        
    def __del__(self):
        """
        close serial port on destruction of object
        """
        self.port.close()

        
    def encode_ubx(self, Cls, ID, length = 0, payload = {}):
        """
        generating UBX commands for ublox neo 6m gps
        """
        
        try:
            # retrieve class and ID specifier
            cmd_Cls = cmd_ref[Cls]['Cls']
            cmd_ID = cmd_ref[Cls]['command'][ID]['ID']
        except KeyError:
            print('Unknown command specifier.')
            return

        # begin message    
        message = struct.pack('<BB', SYNC_CHAR_1, SYNC_CHAR_2)

        # polling messages with length zero
        if length == 0:
            package = struct.pack('<BBH', cmd_Cls, cmd_ID, length)
            ck_a, ck_b = self.checksum(package)
            cksm = struct.pack('<BB', ck_a, ck_b)
            message = message + package + cksm
        else:
            # configuration messages
            # checking length:
            try:
                length_dict = cmd_ref[Cls]['command'][ID]['length']
            except KeyError:
                print('Command not yet implemented in command reference')
                return
            if length != length_dict:
                print('Command length does not match reference. Specified incorrectly or if command can have different lengths not yet implemented')
                return
            else:
                encoding = '<BBH' + cmd_ref[Cls]['command'][ID]['encoding']
                # checking command parameters:
                try:
                    parameter_reference = cmd_ref[Cls]['command'][ID]['payload']
                    payload_msg = []
                    i = 0
                    for parameter in parameter_reference:
                        payload_msg.append(payload[parameter])
                except KeyError:
                    print('Specified payload does not match command reference')
                    return
                package = struct.pack(encoding, cmd_Cls, cmd_ID, length, *payload_msg)
                ck_a, ck_b = self.checksum(package)
                cksm = struct.pack('<BB', ck_a, ck_b)
                message = message + package + cksm
            
        return message


    def checksum(self, package):
        """
        generating checksum for ublox neo 6m UBX commands
        calculated ofer class, id, length, and payload, excluding synch chars and checksum field
        """
        
        ck_a = 0
        ck_b = 0
        for i in package:
            ck_a = ck_a +i
            ck_b = ck_b + ck_a
        ck_a = ck_a % 256
        ck_b = ck_b % 256
        
        return ck_a, ck_b


    def verify_checksum(self, message):
        """
        Compute checksum of message and compare to the transmitted checksum value to verify that the message was transmitted correctly
        """
        
        msg_ck_a = message[-2]
        msg_ck_b = message[-1]
        msg_length = struct.unpack('<H', message[4:6])[0]
        package = message[2:-2]
        ck_a, ck_b = self.checksum(package)
        if ck_a == msg_ck_a and ck_b == msg_ck_b:
            return True
        else:
            return False


    def get_ubx(self):
        """
        read data from serial to detect UBX messages issued by ublox neo 6m GPS and parse data
        """
        
        # read lines over serial until detection of header sequence
        header = struct.pack('<BB', SYNC_CHAR_1, SYNC_CHAR_2)
        for i in range(10):
            line = self.port.readline()
            if  header in line:
                break

        # remove NMEA sequence
        end_index = line.find(b'$GP')
        line = line[:end_index]
        
        # find indices of all ubx header sequences 
        indices = [m.start() for m in re.finditer(header, line)]
        
        # verify that each detected header sequence is followed by a correct Cls-ID sequence
        for index in indices:
            try:
                ubx_cls = cls_ref[line[index+2]]
                ubx_id = id_ref[ubx_cls][line[index+3]]
            except:
                indices.remove(index)
        if indices == []:
            print('No valid header+Cls-ID sequence.')
            return
        
        # split line into individual messages
        messages = []
        if len(indices) == 1:
            message = line[indices[0]:]
            messages.append(message)
        else:
            indices.append(len(line))
            for i in range(len(indices)-1):
                message = line[indices[i]:indices[i+1]]
                messages.append(message)

        # decode message and check for acknowledgement
        for i in range(len(messages)):
            message = decode_ubx(messages[i])
            if message[0] == 'ACK':
                messages.remove(messages[i])
                self.ubx_ack(message)
            else:
                messages[i] = message
                
        return messages


    def ubx_ack(self, message):
        """
        process ublox neo 6m GPS UBX acknowledgement messages
        """
        
        ack_cls = message[0]
        ack_id = message[1]
        msg_cls = cls_ref[message[3]['clsID']]
        msg_id = id_ref[msg_cls][message[3]['msgID']]
        if ack_id == 'ACK':
            print('Message acknowledged, Cls: {}, ID: {}'.format(msg_cls, msg_id))
        if ack_id == 'NAK':
            print('Message not acknowledged, Cls: {}, ID: {}'.format(msg_cls, msg_id))


    def decode_ubx(self, message):
        """
        Verify correct format of ublox neo 6m UBX message and decode values.
        """

        # get raw auxiliary values
        try:
            s_c_1 = message[0]
            s_c_2 = message[1]
            msg_cls = message[2]
            msg_id = message[3]
            msg_length = struct.unpack('<H', message[4:6])[0]
            msg_ck_a = message[-2]
            msg_ck_b = message[-1]
            msg_payload = message[6:-2]
        except:
            print('Message does not match ublox neo 6m UBX protocol, cannot decode.')
            return

        # verify sync characters
        if not (s_c_1 == SYNC_CHAR_1 and s_c_2 == SYNC_CHAR_2):
            print('Sync characters do not match ublox neo 6m UBX protocol.')
            return

        else:
            # extract message class and id
            try:
                msg_cls = cls_ref[msg_cls]
                msg_id = id_ref[msg_cls][msg_id]
            except KeyError:
                print('Class and/or ID do not match reference. Message is incorrect or not yet implemented.')
                return

            # verify that length of payload matches reference
            if msg_length != cmd_ref[msg_cls]['command'][msg_id]['length']:
                print('Length of payload does not match reference. Message is incorrect or not yet implemented.')
                return

            # verify correct checksum
            if not verify_checksum(message):
                print('Checksum is incorrect.')
                return

            # verify payload format
            # in python 3.6 and later, dictionaries are ordered but raspbian is running python 3.5
            if not (sys.version_info[0] >= 3 and sys.version_info[1] >= 6):
                payload = OrderedDict()
            else:
                payload = {}
            try:
                parameters = cmd_ref[msg_cls]['command'][msg_id]['payload']
            except KeyError:
                print('Payload parameters not yet implemented for message. Class: {}, ID: {}, Length: {}'.format(msg_cls, msg_id, msg_length))
                return

            # decode payload
            msg_decode = cmd_ref[msg_cls]['command'][msg_id]['encoding']
            msg_payload = struct.unpack(msg_decode, msg_payload)
            for i in range(len(parameters)):
                parameter = parameters[i]
                payload[parameter] = msg_payload[i]

            return msg_cls, msg_id, msg_length, payload


    def ubx_flags(self, flags):
        """
        check basic format requirements for bitfield strings used for flags in ublox neo 6m ubx commands (X1, X2, X4)
        bitfields are encoded as integers to be packed for transmission in struct.pack commands.
        """
        
        if not len(flags) in [8,16,32]:
            print('Length of supplied bitfield string is incorrect.')
            return
        if type(flags) != str:
            print('Supplied data type for bitfield string is incorrect')
            return
        return int(flags, 2)

    def send_ubx(self, message):
        """
        send UBX commands through serial connection
        """
        self.port.write(message)



"""
#print(decode_ubx(b"\xb5b\x06;,\x00\x01\x06\x00\x00\x00\x90\x02\x00\xe8\x03\x00\x00\x10'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00,\x01\x00\x00O\xc1\x03\x00\x86\x02\x00\x00\xfe\x00\x00\x00d@\x01\x00\x93\x95"))

lp_msg = encode_ubx('CFG', 'RXM', 2, {"reserved1":8, "lpMode":1})


pm2_msg = encode_ubx('CFG', 'PM2', 44, {'version':1,'reserved1':6,'reserved2':0,'reserved3':0,'flags':167936,
                                        'updatePeriod':10000,'searchPeriod':1000,'gridOffset':0,'onTime':0,
                                        'minAcqTime':0,'reserved4':300,'reserved5':0,'reserved6':246095,
                                        'reserved7':646,'reserved8':254,'reserved9':0,'reserved10':0,
                                        'reserved11':82020})
pmreq_msg = encode_ubx('RXM', 'PMREQ', 8, {'duration':0, 'flags':ubx_flags('00000000000000000000000000000010')})
#print(decode_ubx(b'\xb5b\x05\x01\x02\x00\x06;Ir'))

pm2_test = encode_ubx('CFG','PM2')
neo6m = serial.Serial('/dev/ttyS0', 9600)
neo6m.write(pmreq_msg)
answer = get_ubx(neo6m)
print(answer)
"""
