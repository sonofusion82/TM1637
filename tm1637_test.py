from tm1637 import TM1637
import time

t = TM1637(23, 24)

for i in range(0, 100):
    t.setBrightness(0x08 | (i*7/100))
    t.showNumberDec(i)
    time.sleep(0.02)


digits = bytearray(4)
digits[0] = t.encodeDigit(1)
digits[1] = t.encodeDigit(2) | 0x80
digits[2] = t.encodeDigit(3)
digits[3] = t._SEG_A | t._SEG_B | t._SEG_F | t._SEG_G

t.setSegments(digits)

