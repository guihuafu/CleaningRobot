TARGET:=libinoplc.so
Route := ../CleaningRobot/src
SRC_DIR := $(Route)

INCLUDE += -I./
INCLUDE += -I../CleaningRobot
INCLUDE += -I$(Route)
INCLUDE += -I$(Route)/include
INCLUDE += -I$(Route)/include/automove
INCLUDE += -I$(Route)/include/calibrate
INCLUDE += -I$(Route)/include/manualmove
INCLUDE += -I$(Route)/planmove/automove
INCLUDE += -I$(Route)/planmove/basemove
INCLUDE += -I$(Route)/planmove/manualmove
INCLUDE += -I$(Route)/planmove/mathoperation
INCLUDE += -I$(Route)/planmove/motionpara

CFLAGS :=  -g -Wall -D_DEBUG -fPIC -shared $(INCLUDE)

LDFLAGS :=  

C_SRC := $(wildcard *.c)
CXX_SRC := $(wildcard *.cpp)
CXX_SRC += $(wildcard $(SRC_DIR)/*.cpp)
CXX_SRC += $(wildcard $(SRC_DIR)/calibration/*.cpp)
CXX_SRC += $(wildcard $(SRC_DIR)/planmove/automove/*.cpp)
CXX_SRC += $(wildcard $(SRC_DIR)/planmove/basemove/*.cpp)
CXX_SRC += $(wildcard $(SRC_DIR)/planmove/manualmove/*.cpp)
CXX_SRC += $(wildcard $(SRC_DIR)/planmove/mathoperation/*.cpp)
CXX_SRC += $(wildcard $(SRC_DIR)/planmove/motionpara/*.cpp)

OBJS := $(patsubst %.c,%.o,$(C_SRC)) $(patsubst %.cpp,%.o,$(CXX_SRC))

./$(TARGETDIR)/$(TARGET):$(OBJS)
	gcc $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)
	@echo "Build finshed"
%.o:%.c
	gcc $(CFLAGS) -c -o $@ $<
%.o:%.cpp
	g++ $(CFLAGS) -c -o $@ $< 

clean:
	rm -f $(OBJS) *~ core .depend $(TARGET)
