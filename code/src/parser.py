import struct, time, sys
with open(sys.argv[1], "br") as f:
    data = f.read()
i = 0
while True:
    s = struct.unpack("=7f?3x2ifi3f2l", data[i*68:(i+1)*68])
    print(f"vec3 acc: \tx: {s[0]}; \ty: {s[1]} \tz:{s[2]}")
    print(f"quat rot: \tR: {s[3]}; \tI: {s[4]} \tJ: {s[5]} \tK: {s[6]}")
    print(f"BNO: \treset: {s[7]}; \tmissed: {s[8]}")
    print(f"basic data: \tdir: {s[9]}; \talt: {s[10]}; \tparachute: {s[11]}")
    print(f"BMP: \ttemp: {s[12]}; \tpres: {s[13]}; \tbpres: {s[14]}")
    print(f"time: \tflight: {s[15]}; \tboot: {s[16]}")
    i= i+1
    time.sleep(0.01)
