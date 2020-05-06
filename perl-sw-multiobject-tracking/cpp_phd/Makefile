output: Birth_Weight.o Spawn_Motion_Model.o Spawn_Weights.o Existing_Object_Motion_Model.o PositionSensor.o Ground_Truth.o Update_PHD_components.o Missed_detections.o Detected_Object.o Normalize_weights.o Pruning_and_Merging.o MultiTargetStateExtraction.o run_phd.o 
	g++ Birth_Weight.o Spawn_Motion_Model.o Spawn_Weights.o Existing_Object_Motion_Model.o PositionSensor.o Ground_Truth.o Update_PHD_components.o Missed_detections.o Detected_Object.o Normalize_weights.o Pruning_and_Merging.o MultiTargetStateExtraction.o run_phd.o -std=c++11 -I/usr/include/python2.7 -lpython2.7 -o output -O2 -larmadillo

Birth_Weight.o: Birth_Weight.cpp
	g++ -c Birth_Weight.cpp
Spawn_Motion_Model.o: Spawn_Motion_Model.cpp
	g++ -c Spawn_Motion_Model.cpp
Spawn_Weights.o: Spawn_Weights.cpp
	g++ -c Spawn_Weights.cpp
Existing_Object_Motion_Model.o:Existing_Object_Motion_Model.cpp
	g++ -c Existing_Object_Motion_Model.cpp
PositionSensor.o:PositionSensor.cpp 
	g++ -c PositionSensor.cpp 
Ground_Truth.o:Ground_Truth.cpp
	g++ -c Ground_Truth.cpp 
Update_PHD_components.o:Update_PHD_components.cpp
	g++ -c Update_PHD_components.cpp
Missed_detections.o: Missed_detections.cpp 
	g++ -c Missed_detections.cpp 
Detected_Object.o: Detected_Object.cpp
	g++ -c Detected_Object.cpp
Normalize_weights.o:Normalize_weights.cpp
	g++ -c Normalize_weights.cpp
Pruning_and_Merging.o:Pruning_and_Merging.cpp
	g++ -c Pruning_and_Merging.cpp
MultiTargetStateExtraction.o:MultiTargetStateExtraction.cpp
	g++ -c MultiTargetStateExtraction.cpp
run_phd.o:run_phd.cpp
	g++ -c run_phd.cpp -std=c++11 -I/usr/include/python2.7 -lpython2.7
clean:
	rm *.o output


