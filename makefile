Debug/FRCUserProgram : Debug/src/Robot.o Debug/src/Subsystems/Claw.o Debug/src/Subsystems/Lifter.o
	arm-frc-linux-gnueabi-g++ \
		-L/home/blasting/wpilib/common/current/lib/linux/athena/shared \
		-L/usr/arm-frc-linux-gnueabi/lib \
		-L/home/blasting/wpilib/cpp/current/reflib/linux/athena/shared \
		-L/home/blasting/wpilib/user/cpp/lib -pthread -rdynamic \
		-Wl,-rpath,/opt/GenICam_v3_0_NI/bin/Linux32_ARM,-rpath,/usr/local/frc/lib \
		-o Debug/FRCUserProgram \
		Debug/src/Robot.o Debug/src/Subsystems/Claw.o Debug/src/Subsystems/Lifter.o \
		-lCTRE_Phoenix -lCTRE_PhoenixCCI -lwpi

Debug/src/Robot.o : src/Robot.cpp
	mkdir -p Debug/src
	arm-frc-linux-gnueabi-g++ \
		-std=c++1y \
		-I/home/blasting/wpilib/cpp/current/include \
		-I/home/blasting/build/FRC_Workspace/Timed-CAN-DiffDrive--2018/src \
		-I/home/blasting/wpilib/user/cpp/include \
		-O0 -Og -g3 -Wall -c -fmessage-length=0 -pthread \
		-o Debug/src/Robot.o src/Robot.cpp

Debug/src/Subsystems/Claw.o : src/Subsystems/Claw.cpp
	mkdir -p Debug/src/Subsystems
	arm-frc-linux-gnueabi-g++ \
		-std=c++1y \
		-I/home/blasting/wpilib/cpp/current/include \
		-I/home/blasting/build/FRC_Workspace/Timed-CAN-DiffDrive--2018/src \
		-I/home/blasting/wpilib/user/cpp/include \
		-O0 -Og -g3 -Wall -c -fmessage-length=0 -pthread \
		-o Debug/src/Subsystems/Claw.o src/Subsystems/Claw.cpp

Debug/src/Subsystems/Lifter.o : src/Subsystems/Lifter.cpp
	mkdir -p Debug/src/Subsystems
	arm-frc-linux-gnueabi-g++ \
                -std=c++1y \
                -I/home/blasting/wpilib/cpp/current/include \
                -I/home/blasting/build/FRC_Workspace/Timed-CAN-DiffDrive--2018/src \
                -I/home/blasting/wpilib/user/cpp/include \
                -O0 -Og -g3 -Wall -c -fmessage-length=0 -pthread \
                -o Debug/src/Subsystems/Lifter.o src/Subsystems/Lifter.cpp

clean :
	rm -rf Debug/
