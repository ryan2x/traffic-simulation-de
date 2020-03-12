
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
    this.timer = setInterval(function(){
        var obj1 = trafficObjs.trafficObj[0];
        var val1 = obj1.value === 'red' ? "green" : "red";
        trafficObjs.setTrafficLight(obj1, val1);
    }, 5000);
}

RingNetwork.prototype.destroy = function() {
    if (this.timer) {
        clearInterval(this.timer);
    }
};

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

/**
 * RoundAboutNetwork
 */

function RoundAboutNetwork(canvas) {
    // debugging switches
    var markVehsMerge=false; // for debugging road.mergeDiverge
    var drawVehIDs=false;    // for debugging road.mergeDiverge
    var useSsimpleOD_debug=false;
    var drawRingDirect=false; // draw ring vehicles directly instead gen Traj
    this.debugOptions = {
        markVehsMerge, drawVehIDs, useSsimpleOD_debug, drawRingDirect
    };
    this.buildRoads(canvas);
    this.buildTrafficObjects(canvas);
}

RoundAboutNetwork.prototype.buildRoads = function(canvas) {

    //##################################################################
// overall scaling (critAspectRatio should be consistent with
// width/height in css.#contents)
//##################################################################

//!! also change "isSmartphone=" in updateSim!!

    var isSmartphone=mqSmartphone();

    var refSizePhys=(isSmartphone) ? 90 : 110;  // const; all objects scale with refSizePix


    var critAspectRatio=120./95.; // from css file width/height of #contents

    var refSizePix=Math.min(canvas.height,canvas.width/critAspectRatio);
    var scale=refSizePix/refSizePhys;

    // merging fine tuning
//!! fiddle to optimize de-facto anticipation of merging vehs
// and last stopping in order to prevent crashes while waiting

    var padding=30;         // merge: visib. extension for target by origin vehs
    var paddingLTC=20;      // merge: visib. extension for origin by target vehs
    var fracArmBegin=0.87; // merge begin at fracArmBegin of arm length
    var fracArmEnd=0.92; // merge end at fracArmEnd of arm length

    // the following remains constant
// => road becomes more compact for smaller screens

    var car_length=4.5; // car length in m
    var car_width=2.5; // car width in m
    var truck_length=8; // trucks
    var truck_width=3;
    var laneWidth=4;


// all relative "Rel" settings with respect to refSizePhys, not refSizePix!

    var center_xRel=0.63;
    var center_yRel=-0.55;
    var rRingRel=0.14; // ring size w/resp to refSizePhys
    var lArmRel=0.6;

// geom specification ring

    var center_xPhys=center_xRel*refSizePhys; //[m]
    var center_yPhys=center_yRel*refSizePhys;
    var rRing=rRingRel*refSizePhys; // roundabout radius [m]

// geom specification arms

    var lArm=lArmRel*refSizePhys;
    var r1=(rRing/Math.sqrt(2)-0.5*laneWidth)/(1-0.5*Math.sqrt(2));
    var uc1=lArm-0.25*Math.PI*r1;
    var xc1=(rRing+r1)/Math.sqrt(2);
    var yc1=(rRing+r1)/Math.sqrt(2);
    var x01=xc1+uc1;

    //###############################################################
    // physical (m) roads
    //###############################################################

    var nLanes_arm=1;
    var nLanes_ring=1;

    var mergeBegin=fracArmBegin*lArm; // logical merges
    var mergeEnd=fracArmEnd*lArm;

    // central ring (all in physical coordinates)
    // stitchAngleOffset brings stitch of ring as far upstream of merge as possible

    const stitchAngleOffset = -0.20 * Math.PI;

    function trajRing_x(u){
        var dxPhysFromCenter=rRing*Math.cos(u/rRing+stitchAngleOffset);
        return center_xPhys+dxPhysFromCenter;
    }

    function trajRing_y(u){
        var dyPhysFromCenter=rRing*Math.sin(u/rRing+stitchAngleOffset);
        return center_yPhys+dyPhysFromCenter;
    }


// arms 0 and 1 (ingoing/outgoing east arms)



    function traj0_x(u){
        var dxPhysFromCenter=(u<uc1) ? x01-u : xc1-r1*Math.sin((u-uc1)/r1);
        return center_xPhys+dxPhysFromCenter;
    }

    function traj0_y(u){
        var dyPhysFromCenter=(u<uc1) ? 0.5*laneWidth : yc1-r1*Math.cos((u-uc1)/r1);
        return center_yPhys+dyPhysFromCenter;
    }

    function traj1_x(u){
        return traj0_x(lArm-u);
    }

    function traj1_y(u){
        return -traj0_y(lArm-u)+2*center_yPhys;
    }


// arms 2 and 3 (ingoing/outgoing south arms)

    function traj2_x(u){
        return traj0_y(u)-center_yPhys+center_xPhys;
    }

    function traj2_y(u){
        return -traj0_x(u)+center_xPhys+center_yPhys;
    }

    function traj3_x(u){
        return traj1_y(u)-center_yPhys+center_xPhys;
    }

    function traj3_y(u){
        return -traj1_x(u)+center_xPhys+center_yPhys;
    }


// arms 4 and 5 (ingoing/outgoing west arms)

    function traj4_x(u){
        return -traj0_x(u)+2*center_xPhys;
    }

    function traj4_y(u){
        return -traj0_y(u)+2*center_yPhys;
    }

    function traj5_x(u){
        return -traj1_x(u)+2*center_xPhys;
    }

    function traj5_y(u){
        return -traj1_y(u)+2*center_yPhys;
    }

// arms 6 and 7 (ingoing/outgoing north arms)

    function traj6_x(u){
        return -traj2_x(u)+2*center_xPhys;
    }

    function traj6_y(u){
        return -traj2_y(u)+2*center_yPhys;
    }

    function traj7_x(u){
        return -traj3_x(u)+2*center_xPhys;
    }

    function traj7_y(u){
        return -traj3_y(u)+2*center_yPhys;
    }


    var mergeBegin=fracArmBegin*lArm; // logical merges
    var mergeEnd=fracArmEnd*lArm;

    var speedInit=20; // m/s
    var density=0.00;

//new road(ID,length,laneWidth,nLanes,traj_x,traj_y,
//		       density,speedInit,fracTruck,isRing,doGridding[opt]);

// need addtl. road.setOfframpInfo for roads with diverges, nothing for merges

    const {markVehsMerge, drawVehIDs} = this.debugOptions;

    var mainroad = new road(10,2*Math.PI*rRing,laneWidth,nLanes_ring,
        trajRing_x,trajRing_y,0,0,0,true);

    mainroad.padding=padding; mainroad.paddingLTC=paddingLTC;
    if(markVehsMerge){mainroad.markVehsMerge=true;}
    if(drawVehIDs){mainroad.drawVehIDs=true;}

    // diverge length:
// anything above 0.15*Pi*rRing and 1/4 ring length=0.5*Pi*rRing works

    var divLen=0.25*Math.PI*rRing;

// uLastExits[i] such that FIRST exit at rRing*1.75*PI, rRing*1.25*PI etc
// (+stitchAngleOffset) for optical reasons


    var uLastExits=[];

    uLastExits[0]=rRing*(1.75*Math.PI-stitchAngleOffset)+divLen;
    uLastExits[1]=rRing*(1.25*Math.PI-stitchAngleOffset)+divLen;
    uLastExits[2]=rRing*(0.75*Math.PI-stitchAngleOffset)+divLen;
    uLastExits[3]=rRing*(0.25*Math.PI-stitchAngleOffset)+divLen;


// !! odd roadIDs are offramps!!
    mainroad.setOfframpInfo([1,3,5,7], uLastExits, [true,true,true,true]);
    console.log("mainroad.offrampIDs=",mainroad.offrampIDs);
    console.log("mainroad.offrampLastExits=",mainroad.offrampLastExits);
    mainroad.duTactical=divLen;

    var arm=[];
    arm[0]=new road(0,lArm,laneWidth,nLanes_arm,traj0_x,traj0_y,0,0,0,false);
    arm[1]=new road(1,lArm,laneWidth,nLanes_arm,traj1_x,traj1_y,0,0,0,false);
    arm[2]=new road(2,lArm,laneWidth,nLanes_arm,traj2_x,traj2_y,0,0,0,false);
    arm[3]=new road(3,lArm,laneWidth,nLanes_arm,traj3_x,traj3_y,0,0,0,false);
    arm[4]=new road(4,lArm,laneWidth,nLanes_arm,traj4_x,traj4_y,0,0,0,false);
    arm[5]=new road(5,lArm,laneWidth,nLanes_arm,traj5_x,traj5_y,0,0,0,false);
    arm[6]=new road(6,lArm,laneWidth,nLanes_arm,traj6_x,traj6_y,0,0,0,false);
    arm[7]=new road(7,lArm,laneWidth,nLanes_arm,traj7_x,traj7_y,0,0,0,false);

    var network = [];
    network[0]=mainroad;  // network declared in canvas_gui.js
    for (var i=0; i<arm.length; i++){
        network[i+1]=arm[i];
    }

    for (var i=0; i<arm.length; i++){
        arm[i].padding=padding;
        arm[i].paddingLTC=paddingLTC;
        if(markVehsMerge){arm[i].markVehsMerge=true;}
        if(drawVehIDs){arm[i].drawVehIDs=true;}
    }


//############################################################
// add standing virtual vehicle at the end of the merging arms
// new vehicle (length, width, u, lane, speed, type)
// prepending=unshift;
//############################################################

    for(var i=0; i<8; i+=2){
        arm[i].veh.unshift(new vehicle(0.0, laneWidth, mergeEnd, 0, 0, "obstacle"));//!!!
    }

    this.attrs = {
        rRing, stitchAngleOffset,
        lArm,
        mergeBegin, mergeEnd,
        divLen,
        uLastExits,
    };

    //
    this.mainroad = mainroad;
    this.arm = arm;
    //
    this.network = network;
};

RoundAboutNetwork.prototype.updateModels = function() {
    const {rRing} = this.attrs;
// behavior if driving through ring and merge/diverges (car and trucks)
// |lateral accel| <= comf deceleration b

//!!! not yet implemented
    var v0CarRing=Math.min(IDM_v0, Math.sqrt(longModelCar.b*rRing));
    var v0TruckRing=Math.min(factor_v0_truck*IDM_v0, Math.sqrt(longModelTruck.b*rRing));
    var longModelCarRing=new ACC(v0CarRing,IDM_T,IDM_s0,IDM_a,IDM_b);
    var longModelTruckRing=new ACC(v0TruckRing,factor_T_truck*IDM_T,
        IDM_s0,factor_a_truck*IDM_a,IDM_b);
};

RoundAboutNetwork.prototype.buildTrafficObjects = function(canvas) {
    //############################################
    // traffic objects
    //############################################

    // TrafficObjects(canvas,nTL,nLimit,xRelDepot,yRelDepot,nRow,nCol)
    var trafficObjs=new TrafficObjects(canvas,4,0,0.80,0.25,2,2);

    this.trafficObjs = trafficObjs;
};

RoundAboutNetwork.prototype.updateSim = function(time, dt) {
    var mainroad = this.mainroad;
    var arm = this.arm;

//################################################################
// define routes
// 1=E-arm, ingoing, 3=S-arm, ingoing,  5=W-arm, ingoing, 7=N-arm, ingoing
// 2=E-arm, outgoing, 4=S-arm, outgoing,  6=W-arm, outgoing, 8=N-arm, outgoing
//################################################################

    var route1L=[0,10,3];  // inflow E-arm, left turn
    var route1C=[0,10,5];  // inflow E-arm, straight ahead
    var route1R=[0,10,7];  // inflow E-arm, right turn
    var route1U=[0,10,1];  // inflow E-arm, U-tern
    var route3L=[2,10,5];  // inflow S-arm, left turn
    var route3C=[2,10,7];  // inflow S-arm, straight ahead
    var route3R=[2,10,1];  // inflow S-arm, right turn
    var route5L=[4,10,7];  // inflow W-arm, left turn
    var route5C=[4,10,1];  // inflow W-arm, straight ahead
    var route5R=[4,10,3];  // inflow W-arm, right turn
    var route7L=[6,10,1];  // inflow N-arm, left turn
    var route7C=[6,10,3];  // inflow N-arm, straight ahead
    var route7R=[6,10,5];  // inflow N-arm, right turn

    const {markVehsMerge, useSsimpleOD_debug} = this.debugOptions;

    if(markVehsMerge){
        for (var i=0; i<arm.length; i++){arm[i].revertVehMarkings();}
        mainroad.revertVehMarkings();
    }




    //##############################################################
    // (1) transfer effects from slider interaction and mandatory regions
    // to the vehicles and models:
    // also initialize models for new cars entering at inflow points
    //##############################################################


    // updateModelsOfAllVehicles also selectively sets LCModelMandatory
    // to offramp vehs based on their routes!
    // !! needed even for single-lane roads to trigger diverge actions!

    mainroad.updateModelsOfAllVehicles(longModelCar,longModelTruck,
        LCModelCar,LCModelTruck,
        LCModelMandatory);
    for(var i=0; i<arm.length; i++){
        arm[i].updateModelsOfAllVehicles(longModelCar,longModelTruck,
            LCModelCar,LCModelTruck,
            LCModelMandatory);
    }



    //##############################################################
    // (2) do central simulation update of vehicles
    //##############################################################


    //acceleration (no interaction between roads at this point)
    // !! (motion at the end!)

    mainroad.calcAccelerations();
    //mainroad.updateSpeedPositions();
    for(var i=0; i<arm.length; i++){
        arm[i].calcAccelerations();
        //arm[i].updateSpeedPositions();
    }


    // inflow BC

    // route fractions depend on slider-controlled
    // mainFrac, focusFrac  and leftTurnBias
    // main routes: route1C (=[1,10,6], inflow E-arm, straight ahead)
    //              route5C (=[5,10,2], inflow W-arm, opposite direction)
    // road label 1=inflow E-arm
    // road label 2=outflow E-arm
    // road label 3=inflow S-arm etc

    var q1=0.5*mainFrac*qIn;
    var q5=q1;
    var q3=0.5*(1-mainFrac)*qIn;
    var q7=q3;

    var cFrac=1/3. + 2./3*focusFrac - focusFrac*Math.abs(leftTurnBias);
    var lFrac=(1-cFrac)/2.*(1+leftTurnBias);
    var rFrac=(1-cFrac)/2.*(1-leftTurnBias);
    var clFrac=cFrac+lFrac;

    //console.log("roundabout:updateSim: cFrac=",cFrac," lFrac=",lFrac," rFrac=",rFrac);

    var ran=Math.random();


    var route1In=(ran<cFrac) ? route1C : (ran<clFrac) ? route1L : route1R;
    var route3In=(ran<cFrac) ? route3C : (ran<clFrac) ? route3L : route3R;
    var route5In=(ran<cFrac) ? route5C : (ran<clFrac) ? route5L : route5R;
    var route7In=(ran<cFrac) ? route7C : (ran<clFrac) ? route7L : route7R;

    // override for debugging

    if(useSsimpleOD_debug){
        q1=0.2*qIn; q7=0.5*qIn; q5=q3=0;
        route1In=route1C;route7In=route7C;
    }

    arm[0].updateBCup(q1,dt,route1In);
    arm[2].updateBCup(q3,dt,route3In);
    arm[4].updateBCup(q5,dt,route5In);
    arm[6].updateBCup(q7,dt,route7In);

    // outflow BC

    for(var i=1; i<8; i+=2){
        arm[i].updateBCdown();
    }


    //##############################################################
    // merges into the roundabout ring (respecting prio)
    // template: road.mergeDiverge(newRoad,offset,uStart,uEnd,isMerge,
    //                             toRight,[ignoreRoute,
    //                             respectPrioOther,respectPrioOwn])
    //##############################################################

    mainroad.updateLastLCtimes(dt); // needed on target road for graphical merging
    //mainroad.changeLanes(); // only if multilane;  not needed for diverge

    const {
        rRing, stitchAngleOffset,
        lArm,
        mergeBegin, mergeEnd,
    } = this.attrs;

    arm[0].mergeDiverge(mainroad, (0.25*Math.PI-stitchAngleOffset)*rRing-lArm,
        mergeBegin, mergeEnd, true, false, false,
        respectRingPrio, respectRightPrio);

    arm[2].mergeDiverge(mainroad, (1.75*Math.PI-stitchAngleOffset)*rRing-lArm,
        mergeBegin, mergeEnd, true, false, false,
        respectRingPrio, respectRightPrio);

    arm[6].mergeDiverge(mainroad, (0.75*Math.PI-stitchAngleOffset)*rRing-lArm,
        mergeBegin, mergeEnd, true, false, false,
        respectRingPrio, respectRightPrio);

    arm[4].mergeDiverge(mainroad, (1.25*Math.PI-stitchAngleOffset)*rRing-lArm,
        mergeBegin, mergeEnd, true, false, false,
        respectRingPrio, respectRightPrio);



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

    for(var i=1; i<8; i+=2){arm[i].updateLastLCtimes(dt);} // needed for graph LC


    const {divLen, uLastExits} = this.attrs;

    mainroad.mergeDiverge(arm[1], -uLastExits[0]+divLen,
        uLastExits[0]-divLen, uLastExits[0], false, true);
    mainroad.mergeDiverge(arm[3], -uLastExits[1]+divLen,
        uLastExits[1]-divLen, uLastExits[1], false, true);
    mainroad.mergeDiverge(arm[5], -uLastExits[2]+divLen,
        uLastExits[2]-divLen, uLastExits[2], false, true);
    mainroad.mergeDiverge(arm[7], -uLastExits[3]+divLen,
        uLastExits[3]-divLen, uLastExits[3], false, true);


    // !! motion at the end

    mainroad.updateSpeedPositions();
    for(var i=0; i<arm.length; i++){
        arm[i].updateSpeedPositions();

        // !!! forcibly move vehicles behind virtual obstacle vehicle 0
        // if they cross it (may happen for very low a, T)
        // to avoid bugs (otherwise, the vehicle will orbit perpetually
        // on (traj_x,traj_y) instead of merging)

        if(arm[i].veh.length>=2){
            if(arm[i].veh[1].u>arm[i].veh[0].u-0.1){
                arm[i].veh[1].u=arm[i].veh[0].u-0.1;
            }
        }
    }

    var trafficObjs = this.trafficObjs;

    if(userCanDropObjects&&(!isSmartphone)&&(!trafficObjPicked)){
        trafficObjs.zoomBack();
    }


    //##############################################################
    // debug output
    //##############################################################

    if(false){
        var idTest=812;
        for(var iArm=0; iArm<8; iArm++){
            for(var iveh=0; iveh<arm[iArm].veh.length; iveh++){
                if(arm[iArm].veh[iveh].id==idTest){
                    console.log("time=",time," itime=",itime, " vehID=",idTest,
                        " road=arm",iArm, "iveh=",iveh,
                        " u=",arm[iArm].veh[iveh].u,
                        " veh0.u=",arm[iArm].veh[0].u
                    );
                }
            }
        }
        for(var iveh=0; iveh<mainroad.veh.length; iveh++){
            if(mainroad.veh[iveh].id==idTest){
                console.log("time=",time," itime=",itime, " vehID=",idTest,
                    " road=mainroad, iveh=",iveh,
                    " u=",mainroad.veh[iveh].u
                );
            }
        }



        //if((itime>=165)&&(itime<=168)){
        if(false){
            console.log("\nDebug updateSim: Simulation time=",time,
                " itime=",itime);
            mainroad.writeVehiclesSimple();
            //console.log("\nonramp vehicles, simulation time=",time,":");
            arm[6].writeVehiclesSimple();
            arm[7].writeVehiclesSimple();
        }
    }//debug

};

RoundAboutNetwork.prototype.drawSim = function(options) {

    const {drawRingDirect} = this.debugOptions;

    const trafficObjs = this.trafficObjs;

    if (!trafficObjs) {
        console.log(this);
    }

    const {
        stitchAngleOffset,
    } = this.attrs;

    // trafficObjs.calcDepotPositions(canvas);

    var mainroad = this.mainroad;
    var arm = this.arm;

    // (3) draw mainroad and arms


    var changedGeometry=userCanvasManip || hasChanged||(itime<=1);
    for(var i=0; i<arm.length; i++){
        //console.log("draw: i=",i," arm[i].roadLen=",arm[i].roadLen);
        arm[i].draw(armImg1,armImg2,scale,changedGeometry);
    }
    mainroad.draw(ringImg1,ringImg2,scale,changedGeometry);


    // (4) draw vehicles !! degree of smooth changing: fracLaneOptical

    for(var iveh=0; iveh<mainroad.veh.length; iveh++){
        mainroad.veh[iveh].fracLaneOptical=0; // lower than default 1 [lane]
        mainroad.veh[iveh].dt_LC=10; // sufftly long for special traj road.drawVehGen
    }

    for(var i=1; i<arm.length; i+=2){
        for(var iveh=0; iveh<arm[i].veh.length; iveh++){
            arm[i].veh[iveh].fracLaneOptical=0;
            arm[i].veh[iveh].dt_LC=2;
        }
    }

    // actual drawing

    for(var i=0; i<arm.length; i++){
        arm[i].drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col);
    }

    const {lArm, rRing} = this.attrs;

    if(drawRingDirect){
        mainroad.drawVehicles(carImg,truckImg,obstacleImgs,scale,
            vmin_col,vmax_col);
    }

    else{

        // draw ring vehicles in 8 sectors:
        // sectors 0,2,4,6: merging vehs on otherRoad=arm[sector]
        // sectors 1,3,5,7: no mergings (otherRoad=actualRoad=ring
        // end arm 0 attached at 0.25*Math.PI*rRing

        var uOffset0_merge=lArm-(0.25*Math.PI-stitchAngleOffset)*rRing;
        var du=-stitchAngleOffset*rRing;

        mainroad.drawVehiclesGenTraj(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            0, 1./8*mainroad.roadLen+du, // between stitch
            arm[0], uOffset0_merge);
        mainroad.drawVehiclesGenTraj(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            du+mainroad.roadLen, mainroad.roadLen, // between stitch
            arm[0], uOffset0_merge);
        mainroad.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            1./8*mainroad.roadLen+du, 2./8*mainroad.roadLen+du);

        mainroad.drawVehiclesGenTraj(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            2./8*mainroad.roadLen+du, 3./8*mainroad.roadLen+du,
            arm[6], uOffset0_merge-0.25*mainroad.roadLen);
        mainroad.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            3./8*mainroad.roadLen+du, 4./8*mainroad.roadLen+du);

        mainroad.drawVehiclesGenTraj(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            4./8*mainroad.roadLen+du, 5./8*mainroad.roadLen+du,
            arm[4], uOffset0_merge-0.50*mainroad.roadLen);
        mainroad.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            5./8*mainroad.roadLen+du, 6./8*mainroad.roadLen+du);

        mainroad.drawVehiclesGenTraj(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            6./8*mainroad.roadLen+du, 7./8*mainroad.roadLen+du,
            arm[2], uOffset0_merge-0.75*mainroad.roadLen);
        mainroad.drawVehicles(carImg,truckImg,obstacleImgs,scale,vmin_col,vmax_col,
            7./8*mainroad.roadLen+du, 8./8*mainroad.roadLen+du);
    }

    if(false){
        mainroad.writeVehiclesIDrange(1610, 1630);
        for(var i=0; i<arm.length; i++){
            arm[i].writeVehiclesIDrange(1610, 1630);
        }
    }

    // (5a) draw traffic objects

    if(userCanDropObjects&&(!isSmartphone)){
        trafficObjs.draw(scale);
    }

};
