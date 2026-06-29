import logging
import serial

_LOG = logging.getLogger(__name__)

def _u16_be(x: int) -> bytes:
    return bytes([(x >> 8) & 0xFF, x & 0xFF])

def _crc16_modbus(data: bytes) -> int:                      #Calculate CRC16 for ModBus RTU, returns integer value
    """
    CRC16/MODBUS: init=0xFFFF, poly=0xA001, output little-endian in frame.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def _crc16_modbus_bytes(data: bytes) -> bytes:              #Calculate CRC16 for ModBus RTU, returns byte value
    return _crc16_modbus(data).to_bytes(2,"little")

BAUD_MAP = {
    4800: 0x00,
    9600: 0x01,
    19200: 0x02,
    38400: 0x03,
    57600: 0x04,
    115200: 0x05,
    128000: 0x06,
    256000: 0x07,
}

class RelayWave:
    """
    Waveshare Modbus RTU Relay 32CH - https://www.waveshare.com/wiki/Modbus_RTU_Relay_32CH
     - 32 channels | Modbus RTU over RS485
     - Config Relay, Controll single, register, multiple    
    """

    # deviceAddr: int = 0x01
    # timeout: float= 1.0
    
    # ----------------------------------
    # Constructor
    # ----------------------------------
    def __init__(self, ser, deviceAddr: int = 0x01):
        self.ser = ser

        if deviceAddr < 1 or deviceAddr > 247: # !! Verschieben in Fehlerbehandlung
            raise ValueError("Device address must be between 1 and 247.")
        else:
            self.deviceAddr = deviceAddr

    # ----------------------------------
    # Low Level Helper
    # ----------------------------------
    
    # Build data frame for given function code, register and action value, including CRC
    def _buildDataFrame(self, commandCode: int, firstDataWord: int,  secondDataWord: int, specialAddr: int = 0) -> bytes:
        # Frame: [addr][func][reg_hi][reg_lo][val_hi][val_lo] + CRC(lo,hi)
        # CRC16 over first 6 Bytes, CRC im Frame little-endian (low byte first).
        
        if specialAddr == 1:    #Sending Broadcast message
            devAddr = 0x00
        else:
            devAddr = self.deviceAddr

        head = bytes([devAddr & 0xFF, commandCode & 0xFF]) +_u16_be(firstDataWord) +_u16_be(secondDataWord)
        
        return head + _crc16_modbus_bytes(head)
    
    # Send frame and read response, optionally verify echo of sent frame
    def _sendDataFrame(self, dataFrame: bytes, expect_echo: bool = True) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(dataFrame)
        print("TX:", dataFrame.hex())
        self.ser.flush()

        resp = self.ser.read(12)
        print("RX:", resp.hex())

        '''if expect_echo:
            self._validateResponse(resp, dataFrame[1], dataFrame[2] << 8 | dataFrame[3]) #Validate response with function code and register from sent frame
        '''
        return resp
    
    # Validation of received response, checks length, device address, function code, register and CRC
    def _validateResponse(self, resp: bytes, dataFrame: bytes) -> None:
        functionCode = dataFrame[1]

        if dataFrame[0] != resp[0]:
            raise IOError(f"IOE-00-001: Received response from wrong device address: {resp.hex()}")
        
        if dataFrame[1] != resp[1]:
            raise IOError(f"IOE-00-002: Received response with wrong function code: {resp.hex()}")
        

        # Read Coil Status
        if functionCode == 0x01:  #Function Code "Read Coil Status"
            
            if len(resp) != (3 + resp[2] + 2):      #Verify length response based of data inside of response
                raise IOError(f"IOE-01-001: Received response with invalid length: {resp.hex()}")
            
            if _crc16_modbus_bytes(resp[:-2]) != resp[-2]:
                raise IOError(f"IOE-01-003: Received CRC response mismatch: calculated {_crc16_modbus_bytes(resp[:-2])}, received {resp[-2]}")               
            

        # Control Single Coil
        if functionCode == 0x05:  #Function Code "Single Coil Control"
            
            if len(resp) != 8:      #Verify length of response
                raise IOError(f"IOE-05-001: Received response with invalid length: {resp.hex()}")
            
            if  ((dataFrame[2] << 8) | dataFrame[3]) != ((resp[2] << 8) | resp[3]):         #Verify adress of relay
                raise IOError(f"IOE-05-002: Received response from wrong Relay: expected {((dataFrame[2] << 8) | dataFrame[3]) }, got {((resp[2] << 8) | resp[3])}")
                    
            if dataFrame[-2:] != resp[-2:]:
                raise IOError(f"IOE-05-003: Received CRC response mismatch: calculated {dataFrame[-2]}, received {resp[-2]}")


    # ----------------------------------
    # Read Single Relay Status
    # ----------------------------------
    def ReadRelay(self, reqRelay: int, numChannels: int = 1) -> bool:
        """
        Read status of one relay 
        - True: Relay ON
        - False: Relay OFF
        """

        if reqRelay < 1 or reqRelay > 32:
            raise ValueError("Channel must be between 1 and 32.")
            print("!Value out of range!")
        else:
            register = reqRelay - 1

        dataFrame = self._buildDataFrame(0x01, register, 0x0001)
        
        resp = self._sendDataFrame(dataFrame, expect_echo=False)

        if numChannels > 1:
            data = int.from_bytes(resp[3:7], "little")

            return{
                f"Rel{i+1}": (data >> i) & 1
                for i in range(32)
            }
        else:
            return bool(resp[3] & 0x01)

    
    # ----------------------------------
    # Read Every Status of Relays of one Device
    # ----------------------------------
    def ReadRelayAll(self) -> dict:

        return self.ReadRelay(1, 32)
    

    # ----------------------------------
    # Control Command Relay
    # ----------------------------------
    def ControlRelay(self, relay: int, action: str, verify_echo: bool = True) -> bytes:
        """
        Control relay.
        - channel: 0x000..0x001F (Relay 1-32 -> Reg 0..31) 0x00FF all Relays
        - action: 
            'on' = 0xFF00
            'off' = 0x0000
            'toggle' = 0x5500
        """

        if relay == 0x00FF:
            register = relay
        else:
            if relay < 1 or relay > 32:
                raise ValueError("Channel must be between 1 and 32.")
                print("!Value out of range!")
            else:
                register = relay - 1
        
        actionLower = action.lower()

        if actionLower == 'on':
            actionValue = 0xFF00
        elif actionLower == 'off':
            actionValue = 0x0000
        elif actionLower == 'toggle':
            actionValue = 0x5500
        else:
             raise ValueError("Action must be 'ON', 'OFF', or 'TOGGLE'.")
             print("!Invalid action value!")
    
        dataFrame = self._buildDataFrame(0x05, register, actionValue)
        
        return self._sendDataFrame(dataFrame, expect_echo=verify_echo)
    
    # Method wrapper for better readability of control functions
    def RelayOn(self, relay: int, verify_echo: bool = True) -> bytes:
        return self.ControlRelay( relay, 'on', verify_echo)

    # Method wrapper for better readability of control functions
    def RelayOff(self,relay: int, verify_echo: bool = True) -> bytes:
        return self.ControlRelay(relay, 'off', verify_echo)

    # Method wrapper for better readability of control functions
    def RelayToggle(self,relay: int, verify_echo: bool = True) -> bytes:
        return self.ControlRelay(relay, 'toggle', verify_echo) 

    # Method wrapper for better readability of control functions
    def RelayOnAll(self, verify_echo: bool = True) -> bytes:
        return self.ControlRelay(0x00FF, 'on', verify_echo)

    # Method wrapper for better readability of control functions
    def RelayOffAll(self, verify_echo: bool = True) -> bytes:
        return self.ControlRelay(0x00FF, 'off', verify_echo)

    # Method wrapper for better readability of control functions
    def RelayToggleAll(self, verify_echo: bool = True) -> bytes:
        return self.ControlRelay(0x00FF, 'toggle', verify_echo) 

    # ----------------------------------
    # Set the Baudrate of Device
    # ----------------------------------
    def SetBaudrate(self, baudRate: int) -> bool:
        '''
          Set Baudrate of Device
            Parity: no parity
          
          Baudrate Values
            0x00: 4800
            0x01: 9600
            0x02: 19200
            0x03: 38400
            0x04: 57600
            0x05: 115200
            0x06: 128000
            0x07: 256000
        '''
        try:
            value = BAUD_MAP[baudRate]
        except KeyError:
            raise ValueError(f"Unsupported baudrate: {baudRate}")
        

        dataFrame = self._buildDataFrame(0x06, 0x2000, value, 1)  # 0x2000 command to set Baudrate; 1 for Sending Broadcast
        resp = self._sendDataFrame(dataFrame)

        return True

    # ----------------------------------
    # Set Device Address
    # ----------------------------------
    def SetDeviceAddress (self, newDeviceAddr: int) -> int:
        '''
            Command to set Device to new Address
            startDeviceAddr == known device Address, if only one device is on the bus, 0x0 (Broadcast) can be used
        '''

        dataFrame = self._buildDataFrame(0x06, 0x4000, newDeviceAddr) # 0x4000 command to set Device Address
        resp = self._sendDataFrame(dataFrame)

        result = ((resp[4] << 8) | resp[5])

        if result == newDeviceAddr:
            self.deviceAddr = newDeviceAddr
        else:
            raise RuntimeError("Device address change failed")
        
        return result
    
    # ----------------------------------
    # Read Device Address
    # ----------------------------------
    def ReadDeviceAddress (self):
        '''
            Command to read Device Address
        '''

        dataFrame = self._buildDataFrame(0x03, 0x4000, 0x0001) # 0x4000 command to read Device Address, 0x0001 fixed address
        resp = self._sendDataFrame(dataFrame)

        return (resp[0])
    
    # ----------------------------------
    # Read SW Version
    # ----------------------------------
    def ReadSWVersion (self) -> str:
        '''
            Command to reas SW Version of Device
        '''

        dataFrame = self._buildDataFrame(0x03, 0x8000, 0x0001) # 0x8000 command to read SW Version, 0x0001 fixed address
        resp = self._sendDataFrame(dataFrame)

        val = (resp[3] << 8) | resp[4]

        SWversion = f"V{val // 100}.{val % 100:02d}"

        return SWversion
