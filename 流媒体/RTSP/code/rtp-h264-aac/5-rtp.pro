TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS_RELEASE += -O0
SOURCES += main.c \
    rtp-packet.c \
    rtp-payload.c \
    rtp-h264-pack.c \
    rtp-h264-unpack.c \
    h264-util.c \
    rtp-payload-helper.c \
    rtp-mpeg4-generic-unpack.c \
    rtp-mpeg4-generic-pack.c \
    aac-util.c \
    rtp-payload-helper.c

HEADERS += \
    rtp-header.h \
    rtp-packet.h \
    rtp-util.h \
    rtp-payload-internal.h \
    rtp-payload.h \
    rtp-param.h \
    h264-util.h \
    rtp-profile.h \
    rtp-payload-helper.h \
    aac-util.h \
    rtp-payload-helper.h
