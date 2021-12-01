TEMPLATE = lib
TARGET = math
include($$PWD/libmath.pri)
CONFIG += math-buildlib
CONFIG += qt
CONFIG += warn_on
CONFIG += silent





DEFINES += MATH_LIBRARY

# debug and release mode
win32 {
    CONFIG += debug_and_release 
    CONFIG += build_all
}

DESTDIR = $$PROJECT_LIB_DIR
win32 {
    DLLDESTDIR = $$PROJECT_BIN_DIR
    QMAKE_DISTCLEAN += $$PROJECT_BIN_DIR/$${MATH_LIB_NAME}.dll
}
#include($$PWD/../mysharedlib_config.pri)
#include($$PWD/../mysharedlib.pri)