CXX := arm-frc-linux-gnueabi-g++
CXXFLAGS := \
	   -std=c++1y -O0 -Og -g3 -Wall -c -fmessage-length=0 -pthread \
	   -I$(HOME)/wpilib/cpp/current/include \
	   -I$(HOME)/wpilib/user/cpp/include \
	   -Isrc
LDFLAGS := \
	-L$(HOME)/wpilib/common/current/lib/linux/athena/shared \
	-L/usr/arm-frc-linux-gnueabi/lib \
	-L${HOME}/wpilib/cpp/current/reflib/linux/athena/shared \
	-L${HOME}/wpilib/user/cpp/lib -pthread -rdynamic \
	-Wl,-rpath,/opt/GenICam_v3_0_NI/bin/Linux32_ARM,-rpath,/usr/local/frc/lib

LIBS := -lCTRE_Phoenix -lCTRE_PhoenixCCI -lwpi

OBJECTS := \
	Debug/src/Robot.o \
	Debug/src/Subsystems/Claw.o \
	Debug/src/Subsystems/Lifter.o \
	Debug/src/Subsystems/Buddy.o \
	Debug/src/ADIS16448_IMU/ADIS16448_IMU.o

Debug/FRCUserProgram : $(OBJECTS)
	$(CXX) $(LDFLAGS) \
		-o Debug/FRCUserProgram $(OBJECTS) \
		$(LIBS)

Debug/src/%.o : src/%.cpp
	mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) \
		-o $@ $<

.PHONY: deploy clean
deploy : Debug/FRCUserProgram
	ant

clean :
	rm -rf Debug/
