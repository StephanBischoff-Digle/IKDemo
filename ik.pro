CONFIG += c++11 silent debug
QMAKE_CXXFLAGS += -Werror
QT += widgets
QT += core gui

HEADERS += src/IKWidget.hpp \
					 src/Joint.hpp \
					 src/KinematicChain.hpp \
					 src/Window.hpp \
					 src/AnalyticIK.hpp \
					 src/JacobianIK.hpp \
					 src/JacobianPseudoIK.hpp \
					 src/FABRIK.hpp

SOURCES += src/core.cpp \
					 src/IKWidget.cpp \
					 src/Joint.cpp \
					 src/KinematicChain.cpp \
					 src/Window.cpp \
					 src/AnalyticIK.cpp \
					 src/JacobianIK.cpp \
					 src/JacobianPseudoIK.cpp \
					 src/FABRIK.cpp

OBJECTS_DIR = .obj
MOC_DIR = .moc

TARGET = ik
