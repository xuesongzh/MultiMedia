TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

win32 {
INCLUDEPATH += $$PWD/ffmpeg-4.2.1-win32-dev/include
LIBS += $$PWD/ffmpeg-4.2.1-win32-dev/lib/avformat.lib   \
        $$PWD/ffmpeg-4.2.1-win32-dev/lib/avcodec.lib    \
        $$PWD/ffmpeg-4.2.1-win32-dev/lib/avdevice.lib   \
        $$PWD/ffmpeg-4.2.1-win32-dev/lib/avfilter.lib   \
        $$PWD/ffmpeg-4.2.1-win32-dev/lib/avutil.lib     \
        $$PWD/ffmpeg-4.2.1-win32-dev/lib/postproc.lib   \
        $$PWD/ffmpeg-4.2.1-win32-dev/lib/swresample.lib \
        $$PWD/ffmpeg-4.2.1-win32-dev/lib/swscale.lib
#INCLUDEPATH += $$PWD/SDL2/include
#LIBS += $$PWD/SDL2/lib/x86/SDL2.lib
}

SOURCES += main.cpp \
    commonlooper.cpp \
    dlog.cpp \
    audiocapturer.cpp \
    pushwork.cpp \
    videocapturer.cpp \
    avpublishtime.cpp \
    aacencoder.cpp \
    h264encoder.cpp \
    rtsppusher.cpp

HEADERS += \
    commonlooper.h \
    mediabase.h \
    dlog.h \
    audiocapturer.h \
    timesutil.h \
    pushwork.h \
    videocapturer.h \
    avpublishtime.h \
    aacencoder.h \
    h264encoder.h \
    packetqueue.h \
    rtsppusher.h \
    messagequeue.h
