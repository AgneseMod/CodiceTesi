# This file is used for LoRa and Raspberry pi4B related issues 

import RPi.GPIO as GPIO
import serial
import time

class sx126x:

    M0 = 22
    M1 = 27
    # if the header is 0xC0, then the LoRa register settings dont lost when it poweroff, and 0xC2 will be lost. 
    # cfg_reg = [0xC0,0x00,0x09,0x00,0x00,0x00,0x62,0x00,0x17,0x43,0x00,0x00]
    cfg_reg = [0xC2,0x00,0x09,0x00,0x00,0x00,0x62,0x00,0x12,0x43,0x00,0x00]
    get_reg = bytes(12)
    rssi = False
    addr = 65535
    serial_n = ""
    addr_temp = 0

    #
    # start frequence of two lora module
    #
    # E22-400T22S           E22-900T22S
    # 410~493MHz      or    850~930MHz
    start_freq = 850

    #
    # offset between start and end frequence of two lora module
    #
    # E22-400T22S           E22-900T22S
    # 410~493MHz      or    850~930MHz
    offset_freq = 18

    # power = 22
    # air_speed =2400

    SX126X_UART_BAUDRATE_1200 = 0x00
    SX126X_UART_BAUDRATE_2400 = 0x20
    SX126X_UART_BAUDRATE_4800 = 0x40
    SX126X_UART_BAUDRATE_9600 = 0x60
    SX126X_UART_BAUDRATE_19200 = 0x80
    SX126X_UART_BAUDRATE_38400 = 0xA0
    SX126X_UART_BAUDRATE_57600 = 0xC0
    SX126X_UART_BAUDRATE_115200 = 0xE0

    SX126X_PACKAGE_SIZE_240_BYTE = 0x00
    SX126X_PACKAGE_SIZE_128_BYTE = 0x40
    SX126X_PACKAGE_SIZE_64_BYTE = 0x80
    SX126X_PACKAGE_SIZE_32_BYTE = 0xC0

    SX126X_Power_22dBm = 0x00
    SX126X_Power_17dBm = 0x01
    SX126X_Power_13dBm = 0x02
    SX126X_Power_10dBm = 0x03

    lora_air_speed_dic = {
        1200:0x01,
        2400:0x02,
        4800:0x03,
        9600:0x04,
        19200:0x05,
        38400:0x06,
        62500:0x07
    }

    lora_power_dic = {
        22:0x00,
        17:0x01,
        13:0x02,
        10:0x03
    }

    lora_buffer_size_dic = {
        240:SX126X_PACKAGE_SIZE_240_BYTE,
        128:SX126X_PACKAGE_SIZE_128_BYTE,
        64:SX126X_PACKAGE_SIZE_64_BYTE,
        32:SX126X_PACKAGE_SIZE_32_BYTE
    }

    def __init__(self, serial_num, freq, addr, power, rssi, air_speed=2400,
                 net_id=0, buffer_size=240, crypt=0,
                 relay=False, lbt=False, wor=False):
        self.rssi = rssi
        self.addr = addr
        self.freq = freq
        self.serial_n = serial_num
        self.power = power
        self.image_receive_buffer = {}
        self.image_expected_total = None
        # Initial the GPIO for M0 and M1 Pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.M0,GPIO.OUT)
        GPIO.setup(self.M1,GPIO.OUT)
        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.HIGH)
        time.sleep(0.1)
        
        # The hardware UART of Pi3B+,Pi4B is /dev/ttyS0
        self.ser = serial.Serial(serial_num,9600)
        self.ser.flushInput()
        self.set(freq,addr,power,rssi,air_speed,net_id,buffer_size,crypt,relay,lbt,wor)

    def set(self,freq,addr,power,rssi,air_speed=2400,\
            net_id=0,buffer_size = 240,crypt=0,\
            relay=False,lbt=False,wor=False):
        self.send_to = addr
        self.addr = addr
        # We should pull up the M1 pin when sets the module
        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.HIGH)
        time.sleep(0.1)
        val_m0 = GPIO.input(self.M0)
        val_m1 = GPIO.input(self.M1)
        print(f"[DEBUG] Stato M0 (GPIO{self.M0}): {'HIGH' if val_m0 else 'LOW'}")
        print(f"[DEBUG] Stato M1 (GPIO{self.M1}): {'HIGH' if val_m1 else 'LOW'}")
        
        low_addr = addr & 0xff
        high_addr = addr >> 8 & 0xff
        net_id_temp = net_id & 0xff
        if freq > 850:
            freq_temp = freq - 850
            self.start_freq = 850
            self.offset_freq = freq_temp
        elif freq >410:
            freq_temp = freq - 410
            self.start_freq  = 410
            self.offset_freq = freq_temp
        
        air_speed_temp = self.lora_air_speed_dic.get(air_speed,None)
        # if air_speed_temp != None
        
        buffer_size_temp = self.lora_buffer_size_dic.get(buffer_size,None)
        # if air_speed_temp != None:
        
        power_temp = self.lora_power_dic.get(power,None)
        #if power_temp != None:

        if rssi:
            # enable print rssi value 
            rssi_temp = 0x80
        else:
            # disable print rssi value
            rssi_temp = 0x00        

        # get crypt
        l_crypt = crypt & 0xff
        h_crypt = crypt >> 8 & 0xff
        
        if relay==False:
            self.cfg_reg[3] = high_addr
            self.cfg_reg[4] = low_addr
            self.cfg_reg[5] = net_id_temp
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            # 
            # it will enable to read noise rssi value when add 0x20 as follow
            # 
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            #
            # it will output a packet rssi value following received message
            # when enable eighth bit with 06H register(rssi_temp = 0x80)
            #
            self.cfg_reg[9] = 0x43 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        else:
            self.cfg_reg[3] = 0x01
            self.cfg_reg[4] = 0x02
            self.cfg_reg[5] = 0x03
            self.cfg_reg[6] = self.SX126X_UART_BAUDRATE_9600 + air_speed_temp
            # 
            # it will enable to read noise rssi value when add 0x20 as follow
            # 
            self.cfg_reg[7] = buffer_size_temp + power_temp + 0x20
            self.cfg_reg[8] = freq_temp
            #
            # it will output a packet rssi value following received message
            # when enable eighth bit with 06H register(rssi_temp = 0x80)
            #
            self.cfg_reg[9] = 0x03 + rssi_temp
            self.cfg_reg[10] = h_crypt
            self.cfg_reg[11] = l_crypt
        self.ser.flushInput()

        for i in range(2):
            self.ser.write(bytes(self.cfg_reg))
            r_buff = 0
            time.sleep(0.2)
            if self.ser.inWaiting() > 0:
                time.sleep(0.1)
                r_buff = self.ser.read(self.ser.inWaiting())
                if r_buff[0] == 0xC1:
                    pass
                    # print("parameters setting is :",end='')
                    # for i in self.cfg_reg:
                        # print(hex(i),end=' ')
                        
                    # print('\r\n')
                    # print("parameters return is  :",end='')
                    # for i in r_buff:
                        # print(hex(i),end=' ')
                    # print('\r\n')
                else:
                    pass
                    #print("parameters setting fail :",r_buff)
                break
            else:
                print("setting fail,setting again")
                self.ser.flushInput()
                time.sleep(0.2)
                print('\x1b[1A',end='\r')
                if i == 1:
                    print("setting fail,Press Esc to Exit and run again")
                    # time.sleep(2)
                    # print('\x1b[1A',end='\r')

        GPIO.output(self.M0,GPIO.LOW)
        GPIO.output(self.M1,GPIO.LOW)
        time.sleep(0.1)
    
    def get_settings(self):
        # the pin M1 of lora HAT must be high when enter setting mode and get parameters
        GPIO.output(M1,GPIO.HIGH)
        time.sleep(0.1)
        
        # send command to get setting parameters
        self.ser.write(bytes([0xC1,0x00,0x09]))
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            self.get_reg = self.ser.read(self.ser.inWaiting())
        
        # check the return characters from hat and print the setting parameters
        if self.get_reg[0] == 0xC1 and self.get_reg[2] == 0x09:
            fre_temp = self.get_reg[8]
            addr_temp = self.get_reg[3] + self.get_reg[4]
            air_speed_temp = self.get_reg[6] & 0x03
            power_temp = self.get_reg[7] & 0x03
            
            print("Frequence is {0}.125MHz.",fre_temp)
            print("Node address is {0}.",addr_temp)
            print("Air speed is {0} bps"+ lora_air_speed_dic.get(None,air_speed_temp))
            print("Power is {0} dBm" + lora_power_dic.get(None,power_temp))
            GPIO.output(M1,GPIO.LOW)

#
# the data format like as following
# "node address,frequence,payload"
# "20,868,Hello World"
    def send(self,data):
        GPIO.output(self.M1,GPIO.LOW)
        GPIO.output(self.M0,GPIO.LOW)
        time.sleep(0.1)
        print("invio")
        
        self.ser.write(data)
        print(f"[DEBUG] Inviati {len(data)} bytes sulla seriale")
        # if self.rssi == True:
            # self.get_channel_rssi()
        time.sleep(0.1)


    def receive(self):
        if self.ser.inWaiting() > 0:
            time.sleep(0.5)
            r_buff = self.ser.read(self.ser.inWaiting())

            print("receive message from node address with frequence\033[1;32m %d,%d.125MHz\033[0m"%((r_buff[0]<<8)+r_buff[1],r_buff[2]+self.start_freq),end='\r\n',flush = True)
            print("message is "+str(r_buff[3:-1]),end='\r\n')
            
            # print the rssi
            if self.rssi:
                # print('\x1b[3A',end='\r')
                print("the packet rssi value: -{0}dBm".format(256-r_buff[-1:][0]))
                self.get_channel_rssi()
            else:
                pass
                #print('\x1b[2A',end='\r')

    def get_channel_rssi(self):
        GPIO.output(self.M1,GPIO.LOW)
        GPIO.output(self.M0,GPIO.LOW)
        time.sleep(0.1)
        self.ser.flushInput()
        self.ser.write(bytes([0xC0,0xC1,0xC2,0xC3,0x00,0x02]))
        time.sleep(0.5)
        re_temp = bytes(5)
        if self.ser.inWaiting() > 0:
            time.sleep(0.1)
            re_temp = self.ser.read(self.ser.inWaiting())
        if re_temp[0] == 0xC1 and re_temp[1] == 0x00 and re_temp[2] == 0x02:
            print("the current noise rssi value: -{0}dBm".format(256-re_temp[3]))
            # print("the last receive packet rssi value: -{0}dBm".format(256-re_temp[4]))
        else:
            # pass
            print("receive rssi value fail")
            # print("receive rssi value fail: ",re_temp)
            
    def send_image(self, image_path, dest_addr=0, freq=433, timeout_ack=1.0, max_retries=5):
        CHUNK_DATA = 55  # 64 - 6(header) - 1(marker) - 2(metadati) = 55
        with open(image_path, "rb") as f:
            image_data = f.read()

        total_chunks = (len(image_data) + CHUNK_DATA - 1) // CHUNK_DATA
        offset_freq = freq - (850 if freq > 850 else 410)

        HEADER_LORA = (
            bytes([dest_addr >> 8]) + bytes([dest_addr & 0xFF]) +
            bytes([offset_freq]) +
            bytes([self.addr >> 8]) + bytes([self.addr & 0xFF]) +
            bytes([self.offset_freq])
        )

        # --- Invio IMG_START ---
        MARKER_START = bytes([0x01])
        start_payload = bytes([total_chunks])
        start_packet = HEADER_LORA + MARKER_START + start_payload
        self.send(start_packet)
        print("[INFO] Inviato pacchetto IMG_START")

        # --- Invio chunk DATA con attesa ACK ---
        MARKER_DATA = bytes([0x02])
        for i in range(total_chunks):
            start = i * CHUNK_DATA
            end = start + CHUNK_DATA
            chunk = image_data[start:end]

            chunk_payload = bytes([i, total_chunks]) + chunk
            chunk_packet = HEADER_LORA + MARKER_DATA + chunk_payload

            retry_count = 0
            ack_received = False
            while not ack_received and retry_count < max_retries:
                self.send(chunk_packet)
                print(f"[INFO] Inviato pacchetto {i+1}/{total_chunks}, in attesa ACK...")
                ack_received = self.wait_for_ack(i, timeout=timeout_ack)
                if not ack_received:
                    retry_count += 1
                    print(f"[WARN] ACK pacchetto {i+1} non ricevuto, ritento {retry_count}/{max_retries}...")
                    time.sleep(0.05)

            if not ack_received:
                print(f"[ERROR] Pacchetto {i+1} non confermato dopo {max_retries} tentativi. Trasmissione interrotta.")
                return False

        # --- Invio IMG_END ---
        MARKER_END = bytes([0x03])
        end_packet = HEADER_LORA + MARKER_END
        self.send(end_packet)
        print("[INFO] Inviato pacchetto IMG_END")
        return True

    def receive_image_chunk(self):
        """Riceve pacchetti immagine in ordine, invia ACK e ricostruisce l'immagine."""
        HEADER_LORA = 6  # dimensione header LoRa

        if not hasattr(self, 'serial_buffer'):
            self.serial_buffer = b''

        # Legge tutti i byte disponibili dalla seriale
        if self.ser.inWaiting() > 0:
            self.serial_buffer += self.ser.read(self.ser.inWaiting())

        processed = False

        while len(self.serial_buffer) >= HEADER_LORA + 1:  # almeno header + marker
            marker = self.serial_buffer[HEADER_LORA]

            # Determina payload minimo per tipo di pacchetto
            if marker == 0x01:  # IMG_START
                min_payload_len = 1
            elif marker == 0x02:  # DATA
                min_payload_len = 2  # index + total + chunk
            elif marker == 0x03:  # IMG_END
                min_payload_len = 0
            else:
                # Marker sconosciuto, scarta byte
                self.serial_buffer = self.serial_buffer[1:]
                continue

            # Controlla se abbiamo abbastanza byte nel buffer
            if len(self.serial_buffer) < HEADER_LORA + 1 + min_payload_len:
                break  # pacchetto incompleto

            payload = self.serial_buffer[HEADER_LORA + 1:]
            self.serial_buffer = b''  # processiamo un pacchetto alla volta

            if marker == 0x01:  # IMG_START
                self.image_receive_buffer = b''
                self.image_expected_total = payload[0]
                print(f"[INFO] Inizio ricezione immagine: {self.image_expected_total} pacchetti attesi")
                processed = True

            elif marker == 0x02:  # DATA
                if len(payload) < 2:
                    break  # pacchetto incompleto
                index = payload[0]
                total = payload[1]
                chunk = payload[2:]
                self.image_receive_buffer += chunk
                print(f"[INFO] Ricevuto pacchetto {index+1}/{total} ({len(chunk)} byte)")
                self.send_ack(index)
                processed = True

            elif marker == 0x03:  # IMG_END
                with open("received_image.jpg", "wb") as f:
                    f.write(self.image_receive_buffer)
                print("[SUCCESS] Immagine salvata come 'received_image.jpg'")
                self.image_receive_buffer = b''
                self.image_expected_total = None
                processed = True

        return processed


    # --- Funzioni di supporto ---
    def send_ack(self, packet_index):
        HEADER_LORA = 6
        MARKER_ACK = bytes([0x04])  # marker identifica direttamente un pacchetto ACK
        payload = bytes([packet_index])  # conferma quale pacchetto

        # Costruisci pacchetto completo
        ack_packet = (
            bytes([self.dest_addr >> 8]) + bytes([self.dest_addr & 0xFF]) +  # destinazione
            bytes([self.addr >> 8]) + bytes([self.addr & 0xFF]) +            # mittente
            bytes([0]) +  # offset freq se serve
            MARKER_ACK +
            payload
        )

        self.send(ack_packet)
        print(f"[INFO] Inviato ACK per pacchetto {packet_index}")



    def wait_for_ack(self, expected_index, timeout=1.0):
        HEADER_LORA = 6
        MARKER_ACK = 0x04  # marker ACK
        start_time = time.time()

        if not hasattr(self, 'serial_buffer'):
            self.serial_buffer = b''

        while time.time() - start_time < timeout:
            if self.ser.inWaiting() > 0:
                self.serial_buffer += self.ser.read(self.ser.inWaiting())

            # Scorri il buffer alla ricerca di pacchetti ACK
            while len(self.serial_buffer) >= HEADER_LORA + 1 + 1:  # header + marker + payload
                if self.serial_buffer[HEADER_LORA] != MARKER_ACK:
                    # marker non trovato, scarta primo byte
                    self.serial_buffer = self.serial_buffer[1:]
                    continue

                # ACK payload = 1 byte (indice pacchetto)
                payload = self.serial_buffer[HEADER_LORA + 1 : HEADER_LORA + 2]
                self.serial_buffer = self.serial_buffer[HEADER_LORA + 2:]  # rimuovi pacchetto dal buffer

                if payload[0] == expected_index:
                    return True  # ACK corretto ricevuto

            time.sleep(0.01)

        return False