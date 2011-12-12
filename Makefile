# 
#
#
#
CXX=g++
LNK=g++
WINDLLMAIN=
# -m32 export ARCHFLAGS ="-arch i386"

HOSTOS=$(shell uname | tr A-Z a-z)
ifeq ($(HOSTOS),darwin)
 HIDAPIPATH=./hidapi/mac
 LIBS=-lptypes -framework IOKit -framework CoreFoundation
 LNFLAGS=-dynamiclib -flat_namespace -undefined warning -L.
 CFLAGS=-arch i386 -Wall -O3 -DAPL=1 -DIBM=0 -DLIN=0
else
 ifeq ($(HOSTOS),linux)
  HIDAPIPATH=./hidapi/linux
  LIBS=-lptypes
  LNFLAGS=-shared -rdynamic -nodefaultlibs -L.
  CFLAGS=-march=i386 -Wall -O3 -DAPL=0 -DIBM=0-D LIN=1 -fvisibility=hidden
 else # windows
#  SETUPAPI_PATH=C:/Program Files/Microsoft SDKs/Windows/v7.1/Lib
  HOSTOS=windows
  HIDAPIPATH=./hidapi/windows
  LIBS=-lptypes -lXPLM -lSetupAPI
#  LNFLAGS=-WD -mn
  LNFLAGS=-m32 -Wl,-O1 -shared -L. -L./SDK/Libraries/Win/
  CFLAGS= -DAPL=0 -DIBM=1 -DLIN=0 -Wall
#  CC=../dm/bin/dmc
#  CXX=../dm/bin/dmc
  WINDLLMAIN=SaitekProPanelsWin.o
#  OBJ=obj
#  HIDOBJ=
#  LNK=link
 endif
endif

#OPTIONS+=-ggdb -D__XPTESTING__ -DDEBUG

# for USB panel checking pass: -DDO_USBPANEL_CHECK
DEFS=-DXPLM200

INCLUDE=-I./include
INCLUDE+=-I./SDK/CHeaders/XPLM
INCLUDE+=-I./include
INCLUDE+=-I./include/ptypes

all:
#	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) overloaded.cpp
ifeq ($(HOSTOS),windows)
	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) SaitekProPanelsWin.cpp
endif
	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) $(HIDAPIPATH)/hid.c
	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) multipanel.cpp
	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) PanelThreads.cpp
	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) radiopanel.cpp
	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) SaitekProPanels.cpp
	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) switchpanel.cpp
	$(CXX) -c $(INCLUDE) $(DEFS) $(CFLAGS) utils.c

#overloaded.o
	$(LNK) -o SaitekProPanels.xpl hid.o multipanel.o PanelThreads.o radiopanel.o SaitekProPanels.o switchpanel.o utils.o $(WINDLLMAIN) $(LNFLAGS) $(LIBS) 


clean:
	$(RM) *.o *.xpl



