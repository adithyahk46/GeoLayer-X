TEMPLATE = app
TARGET = geolayer_bin


DEFINES -= UNICODE
DEFINES -= _UNICODE
win32 {
    LIBS += -lUser32
}

SOURCES += \
    mainwin.cpp \
