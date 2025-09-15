import time
import re
import math
import mysql.connector
from SX127x.LoRa import LoRa
from SX127x.board_config import BOARD
from SX127x.constants import MODE
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

# ---------- OLED SETUP ----------
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial)
device.clear()
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 9)
except:
    font = ImageFont.load_default()

def display_on_oled(hum, temp, ppm, pm25, pm10, lat, lon):
    image = Image.new("1", (device.width, device.height))
    draw = ImageDraw.Draw(image)
    draw.text((0, 0),  f"H: {hum:.1f}%  T: {temp:.1f}C", font=font, fill=255)
    draw.text((0, 11), f"MQ135: {ppm:.0f} ppm", font=font, fill=255)
    draw.text((0, 22), f"PM2.5: {pm25:.2f} µg/m³", font=font, fill=255)
    draw.text((0, 33), f"PM10 : {pm10:.2f} µg/m³", font=font, fill=255)
    draw.text((0, 44), f"Lon: {lon:.6f}" if lon != "NA" else "Lon: N/A", font=font, fill=255)
    draw.text((0, 54), f"Lat: {lat:.6f}" if lat != "NA" else "Lat: N/A", font=font, fill=255)
    device.display(image)

# ---------- DECRYPTION ----------
ENCRYPTION_KEY = 0xAB
def decrypt_data(encrypted_bytes):
    decrypted = ''.join([chr(b ^ ENCRYPTION_KEY) for b in encrypted_bytes])
    for i, c in enumerate(decrypted):
        if c.isdigit() or c == '-' or c == 'N':
            return decrypted[i:]
    return decrypted

# ---------- MQ135 PPM CALCULATION ----------
def calculate_ppm(raw_adc, RLOAD=10.0, RO=26.2):
    if raw_adc <= 0 or raw_adc >= 1023:
        return 0
    rs = ((1023.0 / raw_adc) - 1.0) * RLOAD
    ratio = rs / RO
    A = 116.6020682
    B = -2.769034857
    ppm = A * math.pow(ratio, B)
    return round(ppm, 2)

# ---------- AQI BREAKPOINTS ----------
PM25_BP = [(0.0, 0), (12.0, 50), (35.4, 100), (55.4, 150), (150.4, 200), (250.4, 300), (350.4, 400), (500.4, 500)]
PM10_BP = [(0, 0), (54, 50), (154, 100), (254, 150), (354, 200), (424, 300), (504, 400), (604, 500)]
MQ135_BP = [(0, 0), (50, 50), (100, 100), (150, 150), (200, 200), (300, 300), (400, 400), (500, 500)]

def compute_aqi(conc, breakpoints):
    for i in range(len(breakpoints) - 1):
        Clow, Ilow = breakpoints[i]
        Chigh, Ihigh = breakpoints[i + 1]
        if Clow <= conc <= Chigh:
            return round(((Ihigh - Ilow) / (Chigh - Clow)) * (conc - Clow) + Ilow)
    return None

def aqi_status(aqi):
    if aqi <= 50:
        return "Good"
    elif aqi <= 100:
        return "Moderate"
    elif aqi <= 150:
        return "Unhealthy for Sensitive Groups"
    elif aqi <= 200:
        return "Unhealthy"
    elif aqi <= 300:
        return "Very Unhealthy"
    else:
        return "Hazardous"

# ---------- DB INSERT ----------
def store_to_db(data):
    try:
        conn = mysql.connector.connect(
            host="localhost",
            user="airuser",          
            password="12345",  
            database="air_quality"
        )
        cursor = conn.cursor()
        sql = """
            INSERT INTO sensor_data (
                humidity, temperature, mq135_raw, ppm, pm25, pm10, lat, lon,
                aqi_pm25, aqi_pm10, aqi_mq135, overall_aqi, status
            ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
        """
        cursor.execute(sql, data)
        conn.commit()
        cursor.close()
        conn.close()
    except mysql.connector.Error as e:
        print("❌ DB Error:", e)

# ---------- LORA RECEIVER ----------
class LoRaReceiver(LoRa):
    def __init__(self, verbose=False):
        super(LoRaReceiver, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

    def on_rx_done(self):
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        decrypted = decrypt_data(payload)
        process_packet(decrypted)
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

# ---------- PROCESS PACKET ----------
def process_packet(data):
    print("\n--- 📡 Received Sensor Data ---")
    print(f"🔓 Decrypted Raw: {data.strip()}")
    try:
        parts = data.strip().split(',')
        if len(parts) == 7:
            humidity = float(parts[0])
            temperature = float(parts[1])
            mq135_raw = int(parts[2])
            pm25 = float(parts[3])
            pm10 = float(parts[4])
            lat = parts[5] if parts[5] != "NA" else "NA"
            lon = parts[6] if parts[6] != "NA" else "NA"

            ppm = calculate_ppm(mq135_raw)
            aqi_pm25 = compute_aqi(pm25, PM25_BP)
            aqi_pm10 = compute_aqi(pm10, PM10_BP)
            aqi_mq135 = compute_aqi(ppm, MQ135_BP)

            aqi_list = list(filter(None, [aqi_pm25, aqi_pm10, aqi_mq135]))
            overall = max(aqi_list) if aqi_list else 0
            status = aqi_status(overall)

            print(f"Humidity     : {humidity:.1f} %")
            print(f"Temperature  : {temperature:.2f} °C")
            print(f"MQ135 PPM    : {ppm:.0f} ppm")
            print(f"PM2.5        : {pm25:.2f} µg/m³")
            print(f"PM10         : {pm10:.2f} µg/m³")
            print(f"GPS          : {lat}, {lon}")
            print(f"📈 AQI PM2.5  : {aqi_pm25}")
            print(f"📈 AQI PM10   : {aqi_pm10}")
            print(f"📈 AQI MQ135  : {aqi_mq135}")
            print(f"🌍 Overall AQI: {overall}")
            print(f"🏷️  Air Quality Status: {status}")

            store_to_db((humidity, temperature, mq135_raw, ppm, pm25, pm10, lat, lon, aqi_pm25, aqi_pm10, aqi_mq135, overall, status))
            display_on_oled(humidity, temperature, ppm, pm25, pm10, lat, lon)
        else:
            print("⚠️ Malformed packet or missing values:", data.strip())
    except Exception as e:
        print("⚠️ Error decoding data:", e)
    print("------------------------------------------------------------")

# ---------- MAIN ----------
if __name__ == "__main__":
    try:
        print("🔌 Configuring LoRa Receiver...")
        for pin in [22, 23, 24, 25]:
            print(f"⚠️ Skipping GPIO.add_event_detect for DIO{pin} (polling mode enabled)")
        lora = LoRaReceiver(verbose=False)
        lora.set_mode(MODE.STDBY)
        lora.set_freq(433.0)
        lora.set_rx_crc(True)
        lora.set_spreading_factor(7)
        lora.reset_ptr_rx()
        lora.set_mode(MODE.RXCONT)

        print("📡 LoRa Receiver Initialized (Polling Mode)")
        print("🔄 Listening for packets (timeout after 40s)...")
        last_received_time = time.time()

        while True:
            if lora.get_irq_flags()["rx_done"]:
                lora.on_rx_done()
                last_received_time = time.time()
                time.sleep(20)
            elif time.time() - last_received_time > 40:
                print("\n⏱️ Timeout: No data received in 40 seconds.")
                last_received_time = time.time()

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user. Exiting...")
    finally:
        lora.set_mode(MODE.SLEEP)
        BOARD.teardown()
        device.clear()
