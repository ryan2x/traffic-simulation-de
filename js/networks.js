
function RingNetwork(roadID, radius, center, options) {
    var roadRadius = radius;
    var mainroadLen=roadRadius*2*Math.PI;
    var center_xPhys = center.x;
    var center_yPhys = center.y;
    var laneWidth = options.laneWidth || 8.0;
    var nLanes_main = options.nLanes_main || 3;
    var isRing = true;
    //
    // on constructing road, road elements are gridded and interna
    // road.traj_xy(u) are generated. The, traj_xy*Init(u) obsolete
    this.traj_x = function (u) {
        return center_xPhys + roadRadius*Math.cos(u/roadRadius);
    };
    this.traj_y = function (u) {
        return center_yPhys + roadRadius*Math.sin(u/roadRadius);
    };
    var mainroad=new road(roadID,mainroadLen,laneWidth,nLanes_main,this.traj_x,this.traj_y,
        density,speedInit,fracTruck,isRing,userCanDistortRoads);
    this.network = [mainroad];
    //
    // introduce stationary detectors
    var detectors=[];
    for(var idet=0; idet<4; idet++){
        detectors[idet]=new stationaryDetector(mainroad,
            (0.125+idet*0.25)*mainroadLen,10);
    }
    this.detectors = detectors;
    //
    var trafficObjs = new TrafficObjects(canvas,2,2,0.40,0.50,3,2);
    this.trafficObjs = trafficObjs;
    // activate traffic light object
    trafficObjs.activate(trafficObjs.trafficObj[0], mainroad, 0);
    // try to set a timer here
    setInterval(function(){
        var obj1 = trafficObjs.trafficObj[0];
        var val1 = obj1.value === 'red' ? "green" : "red";
        trafficObjs.setTrafficLight(obj1, val1);
    }, 5000);
}

RingNetwork.prototype.updateSim = function(time, dt) {
    var mainroad = this.network[0];

    // (2) transfer effects from slider interaction and mandatory regions
    // to the vehicles and models


    mainroad.updateTruckFrac(fracTruck, fracTruckToleratedMismatch);
    mainroad.updateModelsOfAllVehicles(longModelCar,longModelTruck,
        LCModelCar,LCModelTruck,
        LCModelMandatory);
    mainroad.updateDensity(density);

    // (2a) update moveable speed limits

    mainroad.updateSpeedlimits(this.trafficObjs);


    // do central simulation update of vehicles

    mainroad.updateLastLCtimes(dt);
    mainroad.calcAccelerations();
    mainroad.changeLanes();
    mainroad.updateSpeedPositions();

    //if(itime<2){mainroad.writeVehicleLongModels();}
    //if(itime<2){mainroad.writeVehicleLCModels();}

    var detectors = this.detectors;
    for(var iDet=0; iDet<detectors.length; iDet++){
        detectors[iDet].update(time,dt);
    }
};

RingNetwork.prototype.drawSim = function (options) {
    var userCanvasManip = options["userCanvasManip"];
    var hasChanged = options["hasChanged"];
    var itime = options["itime"];
    var scale = options["scale"];
    //
    var mainroad = this.network[0];
    var trafficObjs = this.trafficObjs;

    // (3) draw road and possibly traffic lights afterwards (before vehs)

    var changedGeometry=userCanvasManip || hasChanged||(itime<=1);
    mainroad.draw(roadImg1,roadImg2,scale,changedGeometry);

    // (4) draw vehicles

    mainroad.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col);

    // (5a) draw traffic objects

    if(userCanDropObjects&&(!isSmartphone)){
        trafficObjs.draw(scale);
    }

};

