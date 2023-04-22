#
#    A tool to analyze data created by the electronics package, after recovery of the craft
#    Author: Auri Åhs
#

with open("tele.txt", "br") as f:
    data = f.read()

dat = data.split(b'\x53\x61\x6D')
print(f"runs detected: {len(dat)-1}")
num = 0
for run in dat:
    runData = run.split(b'\x0A')
    try:
        num = num + 1
        sampleRate = float(runData[0].split(b'\00')[1])
        basePressure = float(runData[0].split(b'\00')[3])*100
        runData[0] = runData[0].split(b'\00')[-1]
        print(f'run {num}: {sampleRate} samples per second, calibrated at {basePressure} Pa')
        altitude = [0,0]
        for dataPoint in runData:
            try:
                pres = float(dataPoint)
                alt = -(pres-basePressure)/12
                if alt > altitude[1]:
                    altitude[1] = alt
                if alt < altitude[0]:
                    altitude[0] = alt
            except:
                print("corrupted datapoint")
        print(f'max altitude: {altitude[1]} m, min altitude: {altitude[0]}')
    except:
        print("errors in the data")
