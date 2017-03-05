all: cameraBoard.exe

cameraBoard.exe: cameraBoard.cpp
	g++ -o cameraBoard.exe $< -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann

clean:
	rm cameraBoard.exe
