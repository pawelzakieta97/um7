from um7_my import *
from filter import filter

name1 = 'sensor1'
port1 = 'COM8'


sensor = UM7_my(name1, port1,  baud=115200)
sensor.zerogyros()
sensor.resetekf()
sensor.setRPYBroadcastRate(0)  #setting roll pitch yaw broadcast rate to 1 hz
sensor.setProcBroadcastRate(200)  #setting processed sensor values boadcast rate to 1 hz
i=0
f = filter()
while True:
    #catch sample- blokuje program do czasu pojawienia sie pakietu binarnego
    sensor.catchsample()
    i += 1
    f.update(sensor.getProcessedGyro() * math.pi/180, sensor.getProcessedAcc())
    #f.update(sensor.getProcessedGyro() * math.pi/180, np.array([0, 0, -1]))
    if i == 200:
        print(f.orientation)
        #print('nic')
        #print(sensor.getProcessedAcc())
        i=0
    #print(sensor.getProcessedAcc())
    #print(sensor.getProcessedGyro())
