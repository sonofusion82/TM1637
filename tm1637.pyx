# distutils: language = c++
# distutils: sources = TM1637Display.cpp

from libcpp cimport bool
ctypedef unsigned char uint8_t

cdef extern from "wiringPi.h":
    void wiringPiSetupGpio()

cdef extern from "TM1637Display.h":
    cdef cppclass TM1637Display:
        TM1637Display(uint8_t, uint8_t)
        void setBrightness(uint8_t)
        void setSegments(const uint8_t [], uint8_t, uint8_t)
        void showNumberDec(int num, bool leading_zero, uint8_t length, uint8_t pos)
        uint8_t encodeDigit(uint8_t)

    cdef int SEG_A
    cdef int SEG_B
    cdef int SEG_C
    cdef int SEG_D
    cdef int SEG_E
    cdef int SEG_F
    cdef int SEG_G

# global flag to initialize wiringPi once
isWiringPiInitialized = False

cdef class TM1637:

    _SEG_A = SEG_A
    _SEG_B = SEG_B
    _SEG_C = SEG_C
    _SEG_D = SEG_D
    _SEG_E = SEG_E
    _SEG_F = SEG_F
    _SEG_G = SEG_G

    cdef TM1637Display *thisptr;

    '''Initialize TM1637 using clk and dio pin
    Note: assumes BCM pin number'''
    def __cinit__(self, int clk, int dio):
        global isWiringPiInitialized
        if not isWiringPiInitialized:
            wiringPiSetupGpio()

        self.thisptr = new TM1637Display(clk, dio)

    def __dealloc__(self):
        del self.thisptr

    def setBrightness(self, int brightness):
        self.thisptr.setBrightness(brightness)

    def setSegments(self, segments, pos = 0):
        self.thisptr.setSegments(segments, len(segments), pos)

    def showNumberDec(self, int num, leading_zero = False, int length = 4, int pos = 0):
        self.thisptr.showNumberDec(num, 1 if leading_zero else 0, length, pos)

    def encodeDigit(self, int digit):
        return self.thisptr.encodeDigit(digit)

