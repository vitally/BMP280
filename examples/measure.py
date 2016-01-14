import time
import BMP280
#setting timeout for 7 hours
timeout = time.time() + 60*60*7
file = open('measurments.csv','a')
sensor = BMP280.BMP280()
#writing to csv file every 30 seconds
while True:
    file.write(str(time.time()) + ',' + str(sensor.read_temperature()) + ',' + str(sensor.read_pressure()) + '\n')
    time.sleep(30)
    if time.time() > timeout:
        file.close()
        break
