
function TurnCurve(roadID, rCorner, direction, rampOnSize, center, rampOffLength) {
    let radius = rCorner + laneWidth/2;
    let center_xPhys = center.x;
    let center_yPhys = center.y;
    let arcLength = Math.PI*radius/2;
    let totalLength = arcLength + rampOnSize;
    let nLanes_arm = 1;
    //
    this.totalLength = totalLength;

    let lane_east = function(u, xStart, yStart) {
        let dxPhysFromCenter = u + xStart;
        let dyPhysFromCenter = yStart;
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let lane_west = function(u, xStart, yStart) {
        let dxPhysFromCenter = xStart - u;
        let dyPhysFromCenter = yStart;
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let lane_north = function(u, xStart, yStart) {
        let dxPhysFromCenter = xStart;
        let dyPhysFromCenter = u + yStart;
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let lane_south = function(u, xStart, yStart) {
        let dxPhysFromCenter = xStart;
        let dyPhysFromCenter = yStart - u;
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let curve_west_to_south = function (u) {
        let dxPhysFromCenter = radius*Math.sin(u/radius);
        let dyPhysFromCenter = radius*Math.cos(u/radius);
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let ramp_on_west_to_south = function(u) {
        return lane_west(u, - rampOnSize, radius);
    };
    let ramp_off_west_to_south = function(u) {
        return lane_south(u, radius, 0);
    };
    let curve_west_to_north = function (u) {
        let dxPhysFromCenter = radius*Math.sin(u/radius);
        let dyPhysFromCenter = - radius*Math.cos(u/radius);
        let x = center_xPhys+dxPhysFromCenter;
        let y = center_yPhys+dyPhysFromCenter;
        return {x,y}
    };
    let ramp_on_west_to_north = function(u) {
        return lane_west(u, - rampOnSize, - radius);
    };
    let ramp_off_west_to_north = function(u) {
        return lane_north(u, radius, 0);
    };
    const ramp_on_funcs = {
        "west-to-south": ramp_on_west_to_south,
        "west-to-north": ramp_on_west_to_north,
    };
    const ramp_off_funcs = {
        "west-to-south": ramp_off_west_to_south,
        "west-to-north": ramp_off_west_to_north,
    };
    const curve_funcs = {
        "west-to-south": curve_west_to_south,
        "west-to-north": curve_west_to_north,
    };

    const u_segments = [0, rampOnSize, rampOnSize + arcLength, this.totalLength];
    const u_funcs = [
        ramp_on_funcs[direction],
        curve_funcs[direction],
        ramp_off_funcs[direction],
    ];

    const f_traj_xy = function (u) {
        for (let i=0;i<u_funcs.length;i++) {
            if (u >= u_segments[i] && u <= u_segments[i+1]) {
                return u_funcs[i](u-u_segments[i]);
            }
        }
        let i = u_funcs.length-1;
        return u_funcs[i](u-u_segments[i]);
    };

    const f_traj0_x = function (u) {
        const {x} = f_traj_xy(u);
        return x;
    };
    const f_traj0_y = function (u) {
        const {y} = f_traj_xy(u);
        return y;
    };

    this.road = new road(roadID, totalLength, laneWidth, nLanes_arm,
        f_traj0_x, f_traj0_y,
        0,0,0,false);
}

function StraightRoad(roadID, roadLength, direction, center, options) {
    let {
        numLanes
    } = options;
    if (!numLanes) numLanes = 1;

    let center_xPhys = center.x;
    let center_yPhys = center.y;

    const lane_east = function(u) {
        let x = center_xPhys + u - roadLength/2;
        let y = center_yPhys;
        return {x,y};
    };
    const lane_west = function(u) {
        let x = center_xPhys - u + roadLength/2;
        let y = center_yPhys;
        return {x,y};
    };
    const lane_south = function(u) {
        let x = center_xPhys;
        let y = center_yPhys - u + roadLength/2;
        return {x,y};
    };
    const lane_north = function(u) {
        let x = center_xPhys;
        let y = center_yPhys + u - roadLength/2;
        return {x,y};
    };

    let lane_functions = {
        "east": lane_east,
        "west": lane_west,
        "south": lane_south,
        "north": lane_north,
    };
    let traj_xy = lane_functions[direction];

    let traj_x = function (u) {
        const {x} = traj_xy(u);
        return x;
    };
    let traj_y = function (u) {
        const {y} = traj_xy(u);
        return y;
    };
    //
    this.road=new road(roadID,roadLength,laneWidth, numLanes, traj_x, traj_y,
        density,speedInit,fracTruck, false, userCanDistortRoads);
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

    let rCorner = 12.0;
    let rCorner2 = 30.0;
    let rampOnSize = 9.0;

    let east_road = new StraightRoad(roadID, mainroadLen, "east",
        {x: center_xPhys, y:center_yPhys},
        {numLanes: nLanes_main});

    let south_road = new StraightRoad(roadID, mainroadLen, "south",
        {x: center_xPhys + rCorner + laneWidth/2, y: center_yPhys},
        {numLanes: 1});

    let north_road = new StraightRoad(roadID, mainroadLen, "north",
        {x: center_xPhys + rCorner2 + laneWidth/2, y: center_yPhys},
        {numLanes: 1});

    let mainroad = east_road.road;

    //############################################################
    // add standing virtual vehicle at the end of the merging arms
    // new vehicle (length, width, u, lane, speed, type)
    // prepending=unshift;
    //############################################################
    // mainroad.veh.unshift(new vehicle(0.0, laneWidth, mainroadLen*0.6, 0, 0, "obstacle"));//!!!

    let center1 = {x: center_xPhys, y: center_yPhys - rCorner - laneWidth};
    let center2 = {x: center_xPhys, y: center_yPhys + rCorner2 + laneWidth};
    let curve1 = new TurnCurve(101, rCorner, "west-to-south", rampOnSize, center1);
    let curve2 = new TurnCurve(101, rCorner2, "west-to-north", rampOnSize, center2);

    this.attrs = Object.assign({}, this.attrs, {
        rCorner, rampOnSize,
    });

    this.mainroad = mainroad;
    this.network = [mainroad, curve1.road, curve2.road];
    this.roads = {
        // east_road: east_road.road,
        south_road: south_road.road,
        north_road: north_road.road,
    };
    //
    // introduce stationary detectors
    var detectors=[];
    for(var idet=0; idet<4; idet++){
        detectors[idet]=new stationaryDetector(mainroad,
            (0.125+idet*0.25)*mainroadLen,10);
    }
    this.detectors = detectors;
    //
    var trafficObjs = new TrafficObjects(canvas,3,0,0.40,0.50,3,1);
    this.trafficObjs = trafficObjs;

    let uTrafficLight1 = mainroadLen*0.5 - rampOnSize*0.9;
    let uTrafficLight2 = mainroadLen*0.5 - rCorner - rampOnSize;
    this.attrs = Object.assign({}, this.attrs, {uTrafficLight1, uTrafficLight2});

    let activateTrifficLights = true;
    let trafficLightsAlwaysOn = false;

    if (activateTrifficLights) {
        // activate traffic light object

        let obj1 = trafficObjs.trafficObj[0];
        trafficObjs.activate(obj1, mainroad, uTrafficLight1);
        trafficObjs.setTrafficLight(obj1, "green");
        obj1 = trafficObjs.trafficObj[1];
        trafficObjs.activate(obj1, south_road.road, uTrafficLight2);
        obj1 = trafficObjs.trafficObj[2];
        trafficObjs.activate(obj1, north_road.road, uTrafficLight2);
        //
        if (trafficLightsAlwaysOn) {
            let val1 = "green";
            obj1 = trafficObjs.trafficObj[0];
            trafficObjs.setTrafficLight(obj1, val1);
            obj1 = trafficObjs.trafficObj[1];
            trafficObjs.setTrafficLight(obj1, val1);
            obj1 = trafficObjs.trafficObj[2];
            trafficObjs.setTrafficLight(obj1, val1);
        }
        else {
            // try to set a timer here
            this.timer = setInterval(function(){
                let obj1 = trafficObjs.trafficObj[0];
                let val1 = obj1.value === 'red' ? "green" : "red";
                trafficObjs.setTrafficLight(obj1, val1);
                obj1 = trafficObjs.trafficObj[1];
                let val2 = val1 === 'red' ? "green" : "red";
                trafficObjs.setTrafficLight(obj1, val2);
                obj1 = trafficObjs.trafficObj[2];
                trafficObjs.setTrafficLight(obj1, val2);
            }, 5000);
        }
    }
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

    for (const item of Object.values(this.roads)) {
        item.updateTruckFrac(fracTruck, fracTruckToleratedMismatch);
        item.updateModelsOfAllVehicles(longModelCar,longModelTruck,
            LCModelCar,LCModelTruck,
            LCModelMandatory);
        item.updateDensity(density);
    }

    // (2a) update moveable speed limits

    mainroad.updateSpeedlimits(this.trafficObjs);
    for (const item of Object.values(this.roads)) {
        item.updateSpeedlimits(this.trafficObjs);
    }

    mainroad.calcAccelerations();
    for (const item of Object.values(this.roads)) {
        item.calcAccelerations();
    }
    curve1.calcAccelerations();
    curve2.calcAccelerations();



    // inflow BC
    let q1=0.3*mainFrac*qIn;
    mainroad.updateBCup(q1,dt);
    for (const item of Object.values(this.roads)) {
        item.updateBCup(q1,dt);
    }

    // outflow BC
    // mainroad.updateBCdown();

    mainroad.updateLastLCtimes(dt);
    curve1.updateLastLCtimes(dt);
    curve2.updateLastLCtimes(dt);
    for (const item of Object.values(this.roads)) {
        item.updateLastLCtimes(dt);
    }


    //##############################################################
    // diverges out of the  roundabout ring
    // template: road.mergeDiverge(newRoad,offset,uStart,uEnd,isMerge,
    //                             toRight,[ignoreRoute,respectPrio])
    //##############################################################

    // Besides targetRoad.updateLastLCtimes(dt) as in merge case,
    // addtl provisions necessary:

    // (1) origRoad.setOfframpInfo(...) needs to be added,
    //     best at road cstr time. Iincludes setting origRoad.duTactical>0!
    // (2) origRoad.updateModelsOfAllVehicles(...) needed to trigger diverge,
    //     even for single lane

    const {
        mainroadLen,
        rCorner, rampOnSize,
    } = this.attrs;

    // mergeDiverge(newRoad,offset,uBegin,uEnd, isMerge,toRight,ignoreRoute,prioOther, prioOwn)
    mainroad.mergeDiverge(curve1, rampOnSize - (mainroadLen/2),
        mainroadLen/2-rampOnSize*0.8, mainroadLen/2-rampOnSize*0.1,
        true, false, false,
        true, false);
    mainroad.mergeDiverge(curve2, rampOnSize-(mainroadLen/2),
        mainroadLen/2-rCorner*1.1, mainroadLen/2-rCorner,
        true, true, false,
        true, false);

    // do central simulation update of vehicles

    // // only if multilane;  not needed for diverge
    // mainroad.changeLanes();

    mainroad.updateSpeedPositions();
    for (const [key, item] of Object.entries(this.roads)) {
        item.updateSpeedPositions();
    }
    curve1.updateSpeedPositions();
    curve2.updateSpeedPositions();

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
    mainroad.draw(roadImg1,roadImg1,scale,changedGeometry);
    for (const item of Object.values(this.roads)) {
        item.draw(roadImg1,roadImg1,scale,changedGeometry);
    }
    curve1.draw(roadImg1,roadImg1,scale,changedGeometry);
    curve2.draw(roadImg1,roadImg1,scale,changedGeometry);

    // (4) draw vehicles
// these are not really necessary global variables
    var vmin_col=0; // min speed for speed colormap (drawn in red)
    var vmax_col=100/3.6; // max speed for speed colormap (drawn in blue-violet)

    curve1.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col);
    curve2.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col);

    for (const [key, item] of Object.entries(this.roads)) {
        item.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col);
    }

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

