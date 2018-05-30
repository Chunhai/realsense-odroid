TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp

INSTALLS = target
target.files = putter_detection
target.path = /home/odroid/projects_exe

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

LIBS += -L/usr/local/lib -lrealsense2

DISTFILES += \
    ../Downloads/cascade.xml

HEADERS += \
    cv_helpers.hpp
