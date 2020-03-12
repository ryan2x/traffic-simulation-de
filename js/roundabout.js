
var userCanDropObjects=true;

//#############################################################
// adapt standard slider settings from control_gui.js
// and define variables w/o sliders in this scenario
//#############################################################

// sliders with default inits need not to be reassigned here

respectRingPrio=true; // controlled by a html select element
respectRightPrio=false; // callback: control_gui-handleChangedPriority

// merging fine tuning
//!! fiddle to optimize de-facto anticipation of merging vehs
// and last stopping in order to prevent crashes while waiting

var padding=30;         // merge: visib. extension for target by origin vehs
var paddingLTC=20;      // merge: visib. extension for origin by target vehs
var fracArmBegin=0.87; // merge begin at fracArmBegin of arm length
var fracArmEnd=0.92; // merge end at fracArmEnd of arm length

// vehicle and traffic properties

fracTruck=0.2; // overrides control_gui 0.15
factor_v0_truck=0.9; // truck v0 always slower than car v0 by this factor
                     // (incorporated/updated in sim by updateModels) 
IDM_b=1;

MOBIL_mandat_bSafe=4; // >b, <physical limit
MOBIL_mandat_bThr=0;  
MOBIL_mandat_bias=2; // normal: bias=0.1, rFirst: bias=42
MOBIL_mandat_p=0;  // normal: p=0.2, rFirst: p=0;



qIn=2000./3600;
slider_qIn.value=3600*qIn;
slider_qInVal.innerHTML=3600*qIn+" veh/h";

mainFrac=0.8;
slider_mainFrac.value=100*mainFrac;
slider_mainFracVal.innerHTML=100*mainFrac+"%";

leftTurnBias=0;
//slider_leftTurnBias.value=leftTurnBias;
//slider_leftTurnBiasVal.innerHTML=leftTurnBias;

focusFrac=1;
//slider_focusFrac.value=100*focusFrac;
//slider_focusFracVal.innerHTML=100*focusFrac+"%";

timewarp=8;
slider_timewarp.value=timewarp;
slider_timewarpVal.innerHTML=timewarp +" times";

IDM_v0=50./3.6;
slider_IDM_v0.value=3.6*IDM_v0;
slider_IDM_v0Val.innerHTML=3.6*IDM_v0+" km/h";

IDM_a=0.9; 
slider_IDM_a.value=IDM_a;
slider_IDM_aVal.innerHTML=IDM_a+" m/s<sup>2</sup>";
factor_a_truck=1; // to allow faster slowing down of the uphill trucks

//IDM_T=0.6; // overrides standard settings in control_gui.js
//slider_IDM_T.value=IDM_T;
//slider_IDM_TVal.innerHTML=IDM_T+" s";

// no LC sliders for roundabout



/*######################################################
 Global overall scenario settings and graphics objects
 see onramp.js for more details

 refSizePhys  => reference size in m (generally smaller side of canvas)
 refSizePix   => reference size in pixel (generally smaller side of canvas)
 scale = refSizePix/refSizePhys 
       => roads have full canvas regardless of refSizePhys, refSizePix

 (1) refSizePix=Math.min(canvas.width, canvas.height) determined during run  

 (2) refSizePhys smaller  => all phys roadlengths smaller
  => vehicles and road widths appear bigger for a given screen size 
  => chose smaller for mobile, 

######################################################*
*/

var scenarioString="Roundabout";
console.log("\n\nstart main: scenarioString=",scenarioString);

var simDivWindow=document.getElementById("contents");
var canvas = document.getElementById("canvas"); 
var ctx = canvas.getContext("2d"); // graphics context
canvas.width  = simDivWindow.clientWidth; 
canvas.height  = simDivWindow.clientHeight;
var aspectRatio=canvas.width/canvas.height;


console.log("before addTouchListeners()");
addTouchListeners();
console.log("after addTouchListeners()");


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


//##################################################################
// Specification of physical road geometry and vehicle properties
// If refSizePhys changes, change them all => updateDimensions();
//##################################################################

// the following remains constant 
// => road becomes more compact for smaller screens

var car_length=4.5; // car length in m
var car_width=2.5; // car width in m
var truck_length=8; // trucks
var truck_width=3; 
var laneWidth=4; 


//###############################################################
// physical (m) roads
//###############################################################


var nLanes_arm=1;
var nLanes_ring=1;


//##################################################################
// Specification of logical road network
// template new road(ID,length,laneWidth,nLanes,traj_x,traj_y,
//		     density,speedInit,fracTruck,isRing,doGridding[opt]);
// road with inflow/outflow: just add updateBCup/down at simulation time
// road with passive merge/diverge: nothing needs to be added
// road with active merge (ramp): road.mergeDiverge at sim time
// road with active diverge (mainroad, more generally when routes are relevant): 
//   road.setOfframpInfo at init time and road.mergeDiverge at sim time

//##################################################################


var speedInit=20; // m/s
var density=0.00;

//new road(ID,length,laneWidth,nLanes,traj_x,traj_y,
//		       density,speedInit,fracTruck,isRing,doGridding[opt]);

// need addtl. road.setOfframpInfo for roads with diverges, nothing for merges

var my_network = new RoundAboutNetwork(canvas);


network = network.concat(my_network.network);




//#########################################################
// model initialization (models and methods defined in control_gui.js)
//#########################################################
	
updateModels(); // defines longModelCar,-Truck,LCModelCar,-Truck,-Mandatory

my_network.updateModels();

//####################################################################
// Global graphics specification and image file settings
//####################################################################

var hasChanged=true; // window dimensions have changed (responsive design)

var drawBackground=true; // if false, default unicolor background
var drawRoad=true; // if false, only vehicles are drawn
var userCanvasManip; // true only if user-driven geometry changes

var drawColormap=false;
var vmin_col=0; // min speed for speed colormap (drawn in red)
var vmax_col=70/3.6; // max speed for speed colormap (drawn in blue-violet)


//#########################################################
// The images
//#########################################################


// init background image

var background = new Image();
background.src ='figs/backgroundGrass.jpg'; 
 

// init vehicle image(s)

carImg = new Image();
carImg.src = 'figs/blackCarCropped.gif';
truckImg = new Image();
truckImg.src = 'figs/truck1Small.png';


// init traffic light images

traffLightRedImg = new Image();
traffLightRedImg.src='figs/trafficLightRed_affine.png';
traffLightGreenImg = new Image();
traffLightGreenImg.src='figs/trafficLightGreen_affine.png';


// define obstacle images

obstacleImgNames = []; // srcFiles[0]='figs/obstacleImg.png'
obstacleImgs = []; // srcFiles[0]='figs/obstacleImg.png'
for (var i=0; i<10; i++){
  obstacleImgs[i]=new Image();
  obstacleImgs[i].src = (i==0)
    ? "figs/obstacleImg.png"
    : "figs/constructionVeh"+(i)+".png";
  obstacleImgNames[i] = obstacleImgs[i].src;
}


// init road images

roadImgs1 = []; // road with lane separating line
roadImgs2 = []; // road without lane separating line

for (var i=0; i<4; i++){
    roadImgs1[i]=new Image();
    roadImgs1[i].src="figs/road"+(i+1)+"lanesCropWith.png"
    roadImgs2[i]=new Image();
    roadImgs2[i].src="figs/road"+(i+1)+"lanesCropWithout.png"
}

ringImg1 = new Image();
ringImg1=roadImgs1[nLanes_ring-1];
ringImg2 = new Image();
ringImg2=roadImgs2[nLanes_ring-1];

armImg1 = new Image();
armImg1=roadImgs1[nLanes_arm-1];

armImg2 = new Image();
armImg2=roadImgs2[nLanes_arm-1];


//############################################
// traffic objects
//############################################

// my_network.buildTrafficObjects(canvas);
//
var trafficObjs = my_network.trafficObjs;



//############################################
// run-time specification and functions
//############################################

var time=0;
var itime=0;
var fps=30; // frames per second
var dt=timewarp/fps;




//#################################################################
function updateSim(){
//#################################################################

    // (0) update times and revert vehicle markings if applicable

    time +=dt; // dt depends on timewarp slider (fps=const)
    itime++;
    isSmartphone=mqSmartphone(); // defined in media.js

    my_network.updateSim(time, dt);

}//updateSim




//##################################################
function drawSim() {
//##################################################


    // (0) redefine graphical aspects of road (arc radius etc) using
    // responsive design if canvas has been resized 
    // isSmartphone defined in updateSim
 
    var relTextsize_vmin=(isSmartphone) ? 0.03 : 0.02; //xxx
    var textsize=relTextsize_vmin*Math.min(window.innerWidth,window.innerHeight);


    if(false){
        console.log(" new total inner window dimension: ",
		window.innerWidth," X ",window.innerHeight,
		" (full hd 16:9 e.g., 1120:630)",
		    " canvas: ",canvas.width," X ",canvas.height);
	console.log("isSmartphone=",isSmartphone);

    }


    // (1) define global properties;
    // gridTrajectories only needed if roads can be distorted by mouse

    if ((canvas.width!=simDivWindow.clientWidth)
	||(canvas.height != simDivWindow.clientHeight)){
	hasChanged=true; // only pixel; physical changes in updateSim
	canvas.width  = simDivWindow.clientWidth;
        canvas.height  = simDivWindow.clientHeight;
	aspectRatio=canvas.width/canvas.height;
	refSizePix=Math.min(canvas.height,canvas.width/critAspectRatio);

	scale=refSizePix/refSizePhys; // refSizePhys=constant unless mobile
        //updateDimensions(); // not defined for roundabout


      trafficObjs.calcDepotPositions(canvas);
      if(true){
        console.log("haschanged=true: new canvas dimension: ",
		    canvas.width," X ",canvas.height);
      }
 
/*
	mainroad.gridTrajectories(trajRing_x,trajRing_y);
        arm[0].gridTrajectories(traj0_x,traj0_y);
        arm[1].gridTrajectories(traj1_x,traj1_y);
        arm[2].gridTrajectories(traj2_x,traj2_y);
        arm[3].gridTrajectories(traj3_x,traj3_y);
        arm[4].gridTrajectories(traj4_x,traj4_y);
        arm[5].gridTrajectories(traj5_x,traj5_y);
        arm[6].gridTrajectories(traj6_x,traj6_y);
        arm[7].gridTrajectories(traj7_x,traj7_y);
*/
    }

  // (2) reset transform matrix and draw background
  // (only needed if changes, plus "reminders" for lazy browsers)

  ctx.setTransform(1,0,0,1,0,0);
  if(drawBackground){
    if(hasChanged||(itime<=10) || (itime%50==0) || userCanvasManip
      || (!drawRoad)){
      ctx.drawImage(background,0,0,canvas.width,canvas.height);
    }
  }

 
    my_network.drawSim({});

  // (5b) draw speedlimit-change select box

  ctx.setTransform(1,0,0,1,0,0); 
  drawSpeedlBox();
 
    // (6) draw simulated time

    displayTime(time,textsize);
    //displayMediaProperties(canvas,Math.max(10,textsize));

     // (7) draw the speed colormap

    if(drawColormap){ 
	displayColormap(0.22*refSizePix,
			0.43*refSizePix,
			0.1*refSizePix, 0.2*refSizePix,
			vmin_col,vmax_col,0,100/3.6);
    }

  // may be set to true in next step if changed canvas 
  // or old sign should be wiped away 
  hasChanged=false;

  // revert to neutral transformation at the end!
  ctx.setTransform(1,0,0,1,0,0); 

}// drawSim
 

 //##################################################
// Running function of the sim thread (triggered by setInterval)
//##################################################

function main_loop() {
    updateSim();
    drawSim();
    userCanvasManip=false;
}
 


 //############################################
// start the simulation thread
// THIS function does all the things; everything else 
// only functions/definitions
// triggers:
// (i) automatically when loading the simulation 
// (ii) when pressing the start button 
//  ("myRun=setInterval(main_loop, 1000/fps);")
//############################################

console.log("first main execution");
showInfo();


var myRun=setInterval(main_loop, 1000/fps);



 

 

