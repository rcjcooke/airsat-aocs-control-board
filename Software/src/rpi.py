import spidev
import struct
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 2000000 
spi.mode = 0

CMD_FRAME_FORMAT = "<2Bf4fH"
# 2B = Sync, f = Momentum, H = Propellant, H = Teensy Error Count, 14s = Padded Data, H = Checksum
TELEM_FRAME_FORMAT = "<2BfHH14sH"

# Python Master Error Tracking
pi_rx_errors = 0 
total_transactions = 0

def fletcher16(data: bytes) -> int:
    sum1 = 0
    sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1

def exchange_flight_data(torque, thrust_list):
    global pi_rx_errors, total_transactions
    total_transactions += 1
    
    sync_and_payload = struct.pack("<2Bf4f", 0xAA, 0x55, torque, *thrust_list)
    checksum = fletcher16(sync_and_payload)
    
    tx_frame = struct.pack(CMD_FRAME_FORMAT, 0xAA, 0x55, torque, *thrust_list, checksum)
    rx_bytes = spi.xfer2(list(tx_frame))
    rx_buffer = bytes(rx_bytes)
    
    if rx_buffer[0] == 0xAA and rx_buffer[1] == 0x55:
        calculated_checksum = fletcher16(rx_buffer[:24])
        received_checksum = struct.unpack("<H", rx_buffer[24:26])[0]
        
        if calculated_checksum == received_checksum:
            # Safely extract values out of the telemetry payload
            _, _, momentum, propellant, teensy_rx_errors, _, _ = struct.unpack(TELEM_FRAME_FORMAT, rx_buffer)
            return momentum, propellant, teensy_rx_errors
        else:
            pi_rx_errors += 1
            print("⚠️ Checksum mismatch on Master Rx.")
    else:
        pi_rx_errors += 1
        print("🔄 Searching for Sync Header...")
        
    return None, None, None

try:
    mock_torque = 1.0
    mock_thrusts = [12.0, 11.5, 9.2, 10.0]
    
    print("Beginning flight control communication monitoring...")
    while True:
        momentum, propellant, teensy_errors = exchange_flight_data(mock_torque, mock_thrusts)
        
        if momentum is not None:
            # Output complete diagnostics: physical telemetry alongside bus diagnostics
            print(f"Status | Momentum: {momentum:.2f} | Propellant: {propellant}")
            print(f"Link Quality | Master Rx Errors: {pi_rx_errors} | Teensy Rx Errors: {teensy_errors} (Total Tx: {total_transactions})")
            print("-" * 50)
            
        mock_torque += 0.01
        time.sleep(0.1)

except KeyboardInterrupt:
    spi.close()
    print("\nDiagnostics Stopped.")
