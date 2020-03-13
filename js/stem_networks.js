
function TurnCurve(roadID, rCorner, direction, rampOnSize, center) {
    let radius = rCorner + laneWidth/2;
    let center_xPhys = center.x;
    let center_yPhys = center.y;
    let arcLength = Math.PI*radius/2;
    let totalLength = arcLength + rampOnSize;
    let nLanes_arm = 1;
    //
    this.totalLength = totalLength;

    let curve_west_to_south = function (u) {
        let dxPhysFromCenter = radius*Math.sin(u/radius);
        let dyPhysFromCenter = radius*Math.cos(u/radius);
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let ramp_on_west_to_south = function(u) {
        let dxPhysFromCenter = u - rampOnSize;
        let dyPhysFromCenter = radius;
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let curve_west_to_north = function (u) {
        let dxPhysFromCenter = radius*Math.sin(u/radius);
        let dyPhysFromCenter = - radius*Math.cos(u/radius);
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let ramp_on_west_to_north = function(u) {
        let dxPhysFromCenter = u - rampOnSize;
        let dyPhysFromCenter = - radius;
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    const ramp_on_funcs = {
        "west-to-south": ramp_on_west_to_south,
        "west-to-north": ramp_on_west_to_north,
    };
    const curve_funcs = {
        "west-to-south": curve_west_to_south,
        "west-to-north": curve_west_to_north,
    };

    const f_ramp_on = ramp_on_funcs[direction];
    const f_curve = curve_funcs[direction];

    const f_traj0_x = function (u) {
        if (u < rampOnSize) {
            const {x} = f_ramp_on(u);
            return x;
        }
        else {
            const {x} = f_curve(u-rampOnSize);
            return x;
        }
    };
    const f_traj0_y = function (u) {
        if (u < rampOnSize) {
            const {y} = f_ramp_on(u);
            return y;
        }
        else {
            const {y} = f_curve(u-rampOnSize);
            return y;
        }
    };

    this.road = new road(roadID, totalLength, laneWidth, nLanes_arm,
        f_traj0_x, f_traj0_y,
        0,0,0,false);
}

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
    var mainroadLen=roadRadius*4;


    this.attrs = Object.assign({}, this.attrs, {
        refSizePhys,
        center_xRel, center_yRel, roadRadiusRel,
        center_xPhys, center_yPhys,
        roadRadius,
        mainroadLen,
    });


//##################################################################
// Specification of logical road
//##################################################################

    var isRing=false;  // 0: false; 1: true
    var roadID=12;

    //
    // on constructing road, road elements are gridded and interna
    // road.traj_xy(u) are generated. The, traj_xy*Init(u) obsolete
    this.traj_x = function (u) {
        return center_xPhys + u - mainroadLen/2;
    };
    this.traj_y = function (u) {
        return center_yPhys;
    };
    var mainroad=new road(roadID,mainroadLen,laneWidth,nLanes_main,this.traj_x,this.traj_y,
        density,speedInit,fracTruck,isRing,userCanDistortRoads);

    //############################################################
    // add standing virtual vehicle at the end of the merging arms
    // new vehicle (length, width, u, lane, speed, type)
    // prepending=unshift;
    //############################################################
    mainroad.veh.unshift(new vehicle(0.0, laneWidth, mainroadLen*0.6, 0, 0, "obstacle"));//!!!

    let rCorner = 12.0;
    let rampOnSize = 15.0;
    let center1 = {x: center_xPhys, y: center_yPhys - rCorner - laneWidth};
    let center2 = {x: center_xPhys, y: center_yPhys + rCorner + laneWidth};
    let curve1 = new TurnCurve(101, rCorner, "west-to-south", rampOnSize, center1);
    let curve2 = new TurnCurve(101, rCorner, "west-to-north", rampOnSize, center2);

    this.attrs = Object.assign({}, this.attrs, {
        rCorner, rampOnSize,
    });

    this.network = [mainroad, curve1.road, curve2.road];
    //
    // introduce stationary detectors
    var detectors=[];
    for(var idet=0; idet<4; idet++){
        detectors[idet]=new stationaryDetector(mainroad,
            (0.125+idet*0.25)*mainroadLen,10);
    }
    this.detectors = detectors;
    //
    var trafficObjs = new TrafficObjects(canvas,2,0,0.40,0.50,2,1);
    this.trafficObjs = trafficObjs;
    // activate traffic light object

    let obj1 = trafficObjs.trafficObj[0];
    trafficObjs.activate(obj1, mainroad, mainroadLen*0.3);
    obj1 = trafficObjs.trafficObj[1];
    trafficObjs.activate(obj1, mainroad, mainroadLen*0.6);
    // try to set a timer here
    this.timer = setInterval(function(){
        let obj1 = trafficObjs.trafficObj[0];
        let val1 = obj1.value === 'red' ? "green" : "red";
        trafficObjs.setTrafficLight(obj1, val1);
        obj1 = trafficObjs.trafficObj[1];
        val1 = obj1.value === 'red' ? "green" : "red";
        trafficObjs.setTrafficLight(obj1, val1);
    }, 5000);
};

CrossRoadNetwork.prototype.updateSim = function(time, dt) {
    var mainroad = this.network[0];
    let curve1 = this.network[1];
    let curve2 = this.network[2];

    // (2) transfer effects from slider interaction and mandatory regions
    // to the vehicles and models


    mainroad.updateTruckFrac(fracTruck, fracTruckToleratedMismatch);
    mainroad.updateModelsOfAllVehicles(longModelCar,longModelTruck,
        LCModelCar,LCModelTruck,
        LCModelMandatory);
    mainroad.updateDensity(density);

    // (2a) update moveable speed limits

    mainroad.updateSpeedlimits(this.trafficObjs);

    // inflow BC
    let q1=0.3*mainFrac*qIn;
    mainroad.updateBCup(q1,dt);

    // outflow BC
    // mainroad.updateBCdown();

    const {
        center_xPhys, center_yPhys,
        mainroadLen,
        rCorner, rampOnSize,
    } = this.attrs;

    // mergeDiverge(newRoad,offset,uBegin,uEnd, isMerge,toRight,ignoreRoute,prioOther, prioOwn)
    mainroad.mergeDiverge(curve1, rampOnSize*0.1-(mainroadLen/2),
        mainroadLen/2-rampOnSize*0.8, mainroadLen/2-rampOnSize*0.1, false, true, false,
        true, false);
    mainroad.mergeDiverge(curve2, rampOnSize-(mainroadLen/2),
        mainroadLen/2-rCorner*1.1, mainroadLen/2-rCorner, false, true, false,
        true, false);

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
    let mainroad = this.network[0];
    let curve1 = this.network[1];
    let curve2 = this.network[2];

    var trafficObjs = this.trafficObjs;

    // (3) draw road and possibly traffic lights afterwards (before vehs)

    var changedGeometry=userCanvasManip || hasChanged||(itime<=1);
    curve1.draw(roadImg1,roadImg1,scale,changedGeometry);
    curve2.draw(roadImg1,roadImg1,scale,changedGeometry);
    mainroad.draw(roadImg1,roadImg1,scale,changedGeometry);

    // (4) draw vehicles
// these are not really necessary global variables
    var vmin_col=0; // min speed for speed colormap (drawn in red)
    var vmax_col=100/3.6; // max speed for speed colormap (drawn in blue-violet)

    curve1.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col);
    curve2.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col);

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

