
function CrossRoadNetwork(canvas, options) {
    this.network = [];
    this.detectors = [];
    this.trafficObjs = null;
    this.attrs = {};
    //
    this.buildRoads(canvas, options)
}

CrossRoadNetwork.prototype.buildRoads = function(canvas, options) {
    // var roadRadius = radius;
    // var mainroadLen=roadRadius*2*Math.PI;
    // var center_xPhys = center.x;
    // var center_yPhys = center.y;
    // var laneWidth = options.laneWidth || 8.0;
    // var nLanes_main = options.nLanes_main || 3;
    // var isRing = true;

    var refSizePhys=(isSmartphone) ? 200 : 300;  // constant

    var critAspectRatio=120./95.; // from css file width/height of #contents

    var refSizePix=Math.min(canvas.height,canvas.width/critAspectRatio);
    // control_gui.js
    scale=refSizePix/refSizePhys;

    //
//##################################################################
// Specification of physical road geometry and vehicle properties
// If refSizePhys changes, change them all => updateDimensions();
//##################################################################

// all relative "Rel" settings with respect to refSizePhys, not refSizePix!


    var center_xRel=0.5;
    var center_yRel=-0.54;
    var roadRadiusRel=0.42;

// physical geometry settings [m]

    var center_xPhys=center_xRel*refSizePhys; //[m]
    var center_yPhys=center_yRel*refSizePhys;
    var roadRadius=roadRadiusRel*refSizePhys;
    var mainroadLen=roadRadius*2*Math.PI;


    this.attrs = Object.assign({}, this.attr, {
        refSizePhys,
        center_xRel, center_yRel, roadRadiusRel,
        center_xPhys, center_yPhys,
        roadRadius,
        mainroadLen,
    });


// the following remains constant
// => road becomes more compact for smaller screens

    var laneWidth=8; // remains constant => road becomes more compact for smaller
    var nLanes_main=4;

    var car_length=7; // car length in m
    var car_width=6; // car width in m
    var truck_length=15; // trucks
    var truck_width=7;


//##################################################################
// Specification of logical road
//##################################################################

    var isRing=true;  // 0: false; 1: true
    var roadID=1;
    var speedInit=20; // IC for speed
    var fracTruckToleratedMismatch=0.02; // avoid sudden changes in open systems

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
    this.timer = setInterval(function(){
        var obj1 = trafficObjs.trafficObj[0];
        var val1 = obj1.value === 'red' ? "green" : "red";
        trafficObjs.setTrafficLight(obj1, val1);
    }, 5000);
};

CrossRoadNetwork.prototype.updateSim = function(time, dt) {
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

CrossRoadNetwork.prototype.drawSim = function (canvas, options) {
    const {
        userCanvasManip,
        hasChanged,
        itime,
        scale,
    } = options;
    //
    var mainroad = this.network[0];
    var trafficObjs = this.trafficObjs;

    // (3) draw road and possibly traffic lights afterwards (before vehs)

    var changedGeometry=userCanvasManip || hasChanged||(itime<=1);
    mainroad.draw(roadImg1,roadImg2,scale,changedGeometry);

    // (4) draw vehicles
// these are not really necessary global variables
    var drawColormap=false;
    var vmin_col=0; // min speed for speed colormap (drawn in red)
    var vmax_col=100/3.6; // max speed for speed colormap (drawn in blue-violet)

    mainroad.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col);

    // (5a) draw traffic objects

    if(userCanDropObjects&&(!isSmartphone)){
        trafficObjs.draw(scale);
    }
};

CrossRoadNetwork.prototype.updateDimensions = function(canvas) {
    hasChanged=true;
    canvas.width  = simDivWindow.clientWidth;
    canvas.height  = simDivWindow.clientHeight;
    //
    let aspectRatio=canvas.width/canvas.height;
    let refSizePix=Math.min(canvas.height,canvas.width/critAspectRatio);

    const {refSizePhys, center_xRel,center_yRel, roadRadiusRel} = this.attrs;

    scale=refSizePix/refSizePhys; // refSizePhys=constant unless mobile

    // if viewport or sizePhys changed
    // updateDimensions();
    let center_xPhys=center_xRel*refSizePhys; //[m]
    let center_yPhys=center_yRel*refSizePhys;
    let roadRadius=roadRadiusRel*refSizePhys;
    let mainroadLen=roadRadius*2*Math.PI;

    this.attrs = Object.assign({}, this.attrs, {
        center_xPhys, center_yPhys,
        roadRadius,
        mainroadLen,
    });

    //xxxNew
    this.trafficObjs.calcDepotPositions(canvas);
};

