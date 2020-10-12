import logging
import asyncio
import datetime

from aioconsole import ainput

from bleak import BleakClient, BleakError

read_string_uuid = "00001143-0000-1000-8000-00805f9b34fb" #direccion del servicio de lectura de string
led_control_uuid = "00001150-0000-1000-8000-00805f9b34fb" # direccion del control del led
#definimos variable globales
Ax=0.0
Ay=0.0
Az=0.0
Gx=0.0
Gy=0.0
Gz=0.0
Mx=0.0
My=0.0
Mz=0.0

async def run(address, loop, data_storage, debug=False):
    log = logging.getLogger(__name__)
    stop_event = asyncio.Event()

    if debug:
        import sys
        log.setLevel(logging.DEBUG)
        h = logging.StreamHandler(sys.stdout)
        h.setLevel(logging.DEBUG)
        log.addHandler(h)

    def data_handler(sender, data):
        global Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz
        #log.info("{0}: {1}".format(sender, data))
        data_storage.append((datetime.datetime.utcnow().timestamp(), data))
        aux = bytearray(data)
        datos = stringdata = aux.decode('utf-8')
        msg = datos.split(",")
        Ax=float(msg[0])
        Ay=float(msg[1])
        Az=float(msg[2])
        Gx=float(msg[3])
        Gy=float(msg[4])
        Gz=float(msg[5])
        Mx=float(msg[6])
        My=float(msg[7])
        Mz=float(msg[8])
        print("Accelerometro [xyz]:"+str(Ax)+", "+str(Ay)+", "+str(Az))
        print("Gyroscope [xyz]:"+str(Gx)+", "+str(Gy)+", "+str(Gz))
        print("Magnetometre [xyz]:"+str(Mx)+", "+str(My)+", "+str(Mz))

    async with BleakClient(address, loop=loop) as client:
        while(1):
            x = await client.is_connected()
            log.info("Connected: {0}".format(x))

            log.info("Starting notifications/indications...")
            # Start receiving notifications, which are sent to the `data_handler method`
            await client.start_notify(indication_characteristic_uuid, data_handler)
            
            # wait for instruction to crontrol the led
            keyboard_input = await ainput("Control Led [0,1]: ")
            if keyboard_input== 'x':
                break
            bytes_to_send = bytearray(map(ord, keyboard_input))
            await client.write_gatt_char(led_control_uuid, bytes_to_send, response=True)

            # Send a request to the peripheral to stop sending notifications.
            await client.stop_notify(indication_characteristic_uuid)
            log.info("Stopping notifications/indications...")

    log.info("Disconnected")


if __name__ == "__main__":
    import os

    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    loop = asyncio.get_event_loop()

    output = []
    address = "your mac address"
    loop.run_until_complete(run(address, loop, output, True))

    for item in output:
        epoch = item[0]
        # data here is a bytearray, i.e. a binary text string needing conversion to whatever format you know it
        # to be before using it. Using my test BLE peripheral, the string represents an integer, so I extract the
        # first character and interpret the ascii value as an integer.
        _data_bytearray = item[1]
        data = int(_data_bytearray[0])
        print("Epoch time: {0}, Data: {1}".format(epoch, data))