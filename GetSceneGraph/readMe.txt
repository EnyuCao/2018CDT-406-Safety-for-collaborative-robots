Make sure you have following files in your directory, in order to run the various examples:

1. vrep.py
2. vrepConst.py
3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)
4. vrep_obst_extractor.py 

RUN this demo:
1. Open "A simple vrep model.ttt" with VREP and start the simulation
2. Run vrep_obst_extractor.py 

Note: 
1.install necessary package if required. Such as: 
	sudo pip install shapely

2.The "rosInterfaceControlledBubbleRob" is from V-REP tutorial. It is simple, so start with playing with it. But it doesn't have arm. Delete it when you know you are familiar with V-REP.
3."vrep_obst_extractor.py" need to be adjusted for your use case.

